#include "sdcard.h"

#include <errno.h>
#include <ff.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/check.h>

LOG_MODULE_REGISTER(sdcard, CONFIG_LOG_DEFAULT_LEVEL);

static FATFS fat_fs;

static struct fs_mount_t mount_point = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
};

struct gpio_dt_spec sd_en_gpio_pin = {.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
                                      .pin = 19,
                                      .dt_flags = GPIO_INT_DISABLE};

uint8_t file_count = 0;

#define MAX_PATH_LENGTH 32
static char current_full_path[MAX_PATH_LENGTH];
static char read_buffer[MAX_PATH_LENGTH];
static char write_buffer[MAX_PATH_LENGTH];

#define MAX_AUDIO_FILES 10
uint32_t file_num_array[MAX_AUDIO_FILES + 1];   // [0..9]=10文件大小, [10]=offset 预留

static const char *disk_mount_pt = "/SD:/";

bool sd_enabled = false;

// P14e: 前向声明。scan_audio_files 定义在 mount_sd_card 之后，需在此声明。
static int scan_audio_files(void);

int mount_sd_card(void)
{
    // P13b: skip the sd_en (P0.19) pin on this fly-wire build. The module has
    // no enable pin (power hardwired to 3V3), and P0.19 is claimed by the QSPI
    // peripheral enabled in the v1-spisd overlay (qspi_default uses P0.19 as
    // SCK) - configuring it as a GPIO output fought the QSPI pinctrl.
    LOG_PRINTK("[SD] step 1/4: sd_en skipped (not present on fly-wire module)\n");
    // P13c fix: sd_enabled must ONLY be set once the FULL mount chain succeeds.
    // It previously was set true up front; any later failure left is_sd_on()==true,
    // so manual recording tried to write to an unmounted card -> "fs: invalid file
    // name!!" / "mount point not found" error flood on every audio frame.
    // sd_enabled = true;  <-- REMOVED, set at the successful return below.

    // initialize the sd card
    LOG_PRINTK("[SD] step 2/4: disk_access_init (SPI probe - may block here)\n");
    const char *disk_pdrv = "SD";
    int err = disk_access_init(disk_pdrv);
    LOG_PRINTK("[SD] disk_access_init: %d\n", err);
    if (err) { // reattempt
        k_msleep(1000);
        LOG_PRINTK("[SD] step 2b: disk_access_init retry\n");
        err = disk_access_init(disk_pdrv);
        LOG_PRINTK("[SD] disk_access_init retry: %d\n", err);
        if (err) {
            LOG_ERR("disk_access_init failed");
            return -1;
        }
    }

    LOG_PRINTK("[SD] step 3/4: fs_mount\n");
    mount_point.mnt_point = "/SD:";
    int res = fs_mount(&mount_point);
    if (res == FR_OK) {
        LOG_PRINTK("[SD] mounted OK\n");
    } else if (res == -EBUSY || res == -EEXIST) {
        // P14d: "/SD:" is already mounted. This happens on a soft reset that
        // retained the mount table in RAM, or when a previous mount attempt got
        // past fs_mount but failed at a later step (opendir/move_write_pointer)
        // and is being retried by the mount worker thread. The filesystem object
        // is still registered and usable - so DON'T return -1 here. Returning -1
        // skipped get_file_contents() entirely, which left file_num_array all-zero
        // and made the BLE INFO characteristic report [0,...,0, 0xFFFFFFFF].
        // Continuing re-runs the audio-dir scan below against the live mount.
        LOG_PRINTK("[SD] already mounted (res=%d), continuing with scan\n", res);
    } else {
        LOG_ERR("f_mount failed: %d", res);
        return -1;
    }

    LOG_PRINTK("[SD] step 4/4: audio dir\n");
    res = fs_mkdir("/SD:/audio");

    if (res == FR_OK) {
        LOG_INF("audio directory created successfully");
    } else if (res == FR_EXIST) {
        LOG_INF("audio directory already exists");
    } else {
        LOG_INF("audio directory creation result: %d", res);
    }

    // P14e: 用 fs_stat 直接遍历固定槽位 a01~a10 重建 file_num_array，替代
    // fs_opendir/fs_readdir 扫描。实测 readdir 在该飞线 SPI SD 环境下返回的
    // 是根目录条目（audio/info.txt/a01.txt），把 file_count 误算成 3，随后
    // move_write_pointer(3) 去 stat 不存在的 audio/a03.txt 而失败，导致
    // sd_enabled 恒为 false、录音不落盘。直接 stat 命名文件走的是独立代码
    // 路径（f_stat，非 f_opendir/f_readdir），绕开 readdir 的目录定位问题。
    file_count = scan_audio_files();
    if (file_count < 1) {
        file_count = 1;   // 目录为空，从 a01 开始
    }
    if (file_count > MAX_AUDIO_FILES) {
        file_count = MAX_AUDIO_FILES;   // clamp 到上限
    }

    // 确保写指针槽位文件存在（空目录/空槽位则创建 0 字节占位文件），否则
    // 下面的 move_write_pointer 会 stat 失败并提前 return。
    res = initialize_audio_file(file_count);
    if (res) {
        LOG_ERR("init audio file %d failed: %d", file_count, res);
        return -1;
    }
    LOG_INF("new num files: %d", file_count);

    res = move_write_pointer(file_count);
    if (res) {
        LOG_ERR("error while moving the write pointer");
        return -1;
    }

    move_read_pointer(file_count);

    if (res) {
        LOG_ERR("error while moving the reader pointer\n");
        return -1;
    }
    LOG_INF("file count: %d", file_count);

    struct fs_dirent info_file_entry; // check if the info file exists. if not, generate new info file
    const char *info_path = "/SD:/info.txt";
    res = fs_stat(info_path, &info_file_entry); // for later
    if (res) {
        res = create_file("info.txt");
        LOG_INF("result of info.txt creation: %d ", res);
    }

    // P14f: 无条件清零读偏移。P14c use-after-free 时期可能把垃圾字节写进 info.txt，
    // 使 get_offset() 读到 0x49445541("AUDI") 这种巨量偏移；PC 端下载时据此发 READ
    // size=该偏移，会被固件判 "requested size is too large" 拒绝，录音无法下载。
    // 这里每次挂载都清零，保证 INFO 的 offset 字段恒有效（下载进度不跨复位保留，
    // 本项目手动录音场景可接受）。
    res = save_offset(0);
    if (res) {
        LOG_ERR("reset offset failed: %d", res);
    }

    LOG_INF("result of check: %d", res);

    // P13c fix: card is truly mounted & audio dir ready now - only then report
    // SD as available so is_sd_on() gates recording writes correctly.
    sd_enabled = true;
    LOG_PRINTK("[SD] SD card fully mounted, sd_enabled=true\n");

    return 0;
}

uint32_t get_file_size(uint8_t num)
{
    char *ptr = generate_new_audio_header(num);
    snprintf(current_full_path, sizeof(current_full_path), "%s%s", disk_mount_pt, ptr);
    k_free(ptr);
    struct fs_dirent entry;
    int res = fs_stat(&current_full_path, &entry);
    if (res) {
        LOG_ERR("invalid file in get file size\n");
        return 0;
    }
    return (uint32_t) entry.size;
}

int move_read_pointer(uint8_t num)
{
    char *read_ptr = generate_new_audio_header(num);
    snprintf(read_buffer, sizeof(read_buffer), "%s%s", disk_mount_pt, read_ptr);
    k_free(read_ptr);
    struct fs_dirent entry;
    int res = fs_stat(&read_buffer, &entry);
    if (res) {
        LOG_ERR("invalid file in move read ptr\n");
        return -1;
    }
    return 0;
}

int move_write_pointer(uint8_t num)
{
    char *write_ptr = generate_new_audio_header(num);
    snprintf(write_buffer, sizeof(write_buffer), "%s%s", disk_mount_pt, write_ptr);
    k_free(write_ptr);
    struct fs_dirent entry;
    int res = fs_stat(&write_buffer, &entry);
    if (res) {
        LOG_ERR("invalid file in move write pointer\n");
        return -1;
    }
    return 0;
}

int create_file(const char *file_path)
{
    int ret = 0;
    snprintf(current_full_path, sizeof(current_full_path), "%s%s", disk_mount_pt, file_path);
    struct fs_file_t data_file;
    fs_file_t_init(&data_file);
    ret = fs_open(&data_file, current_full_path, FS_O_WRITE | FS_O_CREATE);
    if (ret) {
        LOG_ERR("File creation failed %d", ret);
        return -2;
    }
    fs_close(&data_file);
    return 0;
}

// P14: 开始一次新的手动录音 —— 分配下一个可用编号文件并切换写指针。
// 策略：优先使用"空文件/已删除"的空闲编号（file_num_array[i]==0），
//       这样永远不会覆盖还有未上传数据的文件（未上传的坚决不删）。
//       若所有编号都非空（极少见），回绕到 1 覆盖最旧的（多为已上传、PC 已删）。
int start_new_recording(void)
{
    // 1. 尝试找一个 size==0 的空闲编号（已上传被删 / 从未使用）
    uint8_t target = 0;
    for (uint8_t i = 1; i <= MAX_AUDIO_FILES; i++) {
        if (file_num_array[i - 1] == 0) {
            target = i;
            break;
        }
    }
    // 2. 没有空闲编号：从当前 file_count 递增，到上限回绕到 1（覆盖最旧）
    if (target == 0) {
        target = (file_count % MAX_AUDIO_FILES) + 1;
    }
    file_count = target;
    LOG_PRINTK("[P14] start recording -> file a%02u.txt\n", file_count);
    int rc = initialize_audio_file(file_count);
    if (rc) {
        LOG_ERR("[P14] init file failed: %d", rc);
        return rc;
    }
    return move_write_pointer(file_count);
}

int read_audio_data(uint8_t *buf, int amount, int offset)
{
    struct fs_file_t read_file;
    fs_file_t_init(&read_file);
    uint8_t *temp_ptr = buf;
    struct fs_dirent entry;

    int rc = fs_open(&read_file, read_buffer, FS_O_READ | FS_O_RDWR);
    rc = fs_seek(&read_file, offset, FS_SEEK_SET);
    rc = fs_read(&read_file, temp_ptr, amount);
    // LOG_PRINTK("read data :");
    // for (int i = 0; i < amount;i++) {
    //     LOG_PRINTK("%d ",temp_ptr[i]);
    // }
    // LOG_PRINTK("\n");
    fs_close(&read_file);

    return rc;
}

int write_to_file(uint8_t *data, uint32_t length)
{
    struct fs_file_t write_file;
    fs_file_t_init(&write_file);
    uint8_t *write_ptr = data;
    fs_open(&write_file, write_buffer, FS_O_WRITE | FS_O_APPEND);
    fs_write(&write_file, write_ptr, length);
    fs_close(&write_file);
    return 0;
}

int initialize_audio_file(uint8_t num)
{
    // P14c: 修复 use-after-free —— 原来先 k_free(header) 再 create_file(header)，
    // create_file 内部 snprintf 会读已释放的 header 内存，路径字符串是垃圾数据，
    // fs_open 失败导致文件创建失败（start_new_recording 连锁失败，录音不落盘）。
    // 正确顺序：先 create_file（会复制 header 到 current_full_path），再释放。
    char *header = generate_new_audio_header(num);
    if (header == NULL) {
        return -1;
    }
    int rc = create_file(header);
    k_free(header);
    return rc;
}

char *generate_new_audio_header(uint8_t num)
{
    if (num > 99)
        return NULL;
    char *ptr_ = k_malloc(14);
    ptr_[0] = 'a';
    ptr_[1] = 'u';
    ptr_[2] = 'd';
    ptr_[3] = 'i';
    ptr_[4] = 'o';
    ptr_[5] = '/';
    ptr_[6] = 'a';
    ptr_[7] = 48 + (num / 10);
    ptr_[8] = 48 + (num % 10);
    ptr_[9] = '.';
    ptr_[10] = 't';
    ptr_[11] = 'x';
    ptr_[12] = 't';
    ptr_[13] = '\0';

    return ptr_;
}

int get_file_contents(struct fs_dir_t *zdp, struct fs_dirent *entry)
{
    if (zdp->mp->fs->readdir(zdp, entry)) {
        return -1;
    }
    if (entry->name[0] == 0) {
        return 0;
    }
    int count = 0;
    file_num_array[count] = entry->size;
    LOG_INF("file numarray %d %d ", count, file_num_array[count]);
    LOG_INF("file name is %s ", entry->name);
    count++;
    while (zdp->mp->fs->readdir(zdp, entry) == 0) {
        if (entry->name[0] == 0) {
            break;
        }
        if (count >= MAX_AUDIO_FILES) {
            break;   // P14: 防止 file_num_array 越界（最多记录 MAX_AUDIO_FILES 个）
        }
        file_num_array[count] = entry->size;
        LOG_INF("file numarray %d %d ", count, file_num_array[count]);
        LOG_INF("file name is %s ", entry->name);
        count++;
    }
    return count;
}

// P14e: 用 fs_stat 遍历固定录音槽位 a01~a10 重建 file_num_array。
// 返回最大已存在槽位编号（1..MAX_AUDIO_FILES），0 表示目录为空。
// 相比 get_file_contents（依赖 fs_readdir），这里直接 stat 命名文件，
// 绕开了 readdir 在该飞线 SPI SD 环境下返回根目录条目的问题。
static int scan_audio_files(void)
{
    int max_used = 0;
    for (uint8_t i = 1; i <= MAX_AUDIO_FILES; i++) {
        char *header = generate_new_audio_header(i);
        char path[MAX_PATH_LENGTH];
        snprintf(path, sizeof(path), "%s%s", disk_mount_pt, header);
        k_free(header);

        struct fs_dirent entry;
        if (fs_stat(path, &entry) == 0) {
            file_num_array[i - 1] = entry.size;
            max_used = i;
        } else {
            file_num_array[i - 1] = 0;
        }
    }
    return max_used;
}
// we should clear instead of delete since we lose fifo structure
int clear_audio_file(uint8_t num)
{
    char *clear_header = generate_new_audio_header(num);
    snprintf(current_full_path, sizeof(current_full_path), "%s%s", disk_mount_pt, clear_header);
    k_free(clear_header);
    int res = fs_unlink(current_full_path);
    if (res) {
        LOG_ERR("error deleting file");
        return -1;
    }

    char *create_file_header = generate_new_audio_header(num);
    k_msleep(10);
    res = create_file(create_file_header);
    k_free(create_file_header);
    if (res) {
        LOG_ERR("error creating file");
        return -1;
    }

    return 0;
}

int delete_audio_file(uint8_t num)
{
    char *ptr = generate_new_audio_header(num);
    snprintf(current_full_path, sizeof(current_full_path), "%s%s", disk_mount_pt, ptr);
    k_free(ptr);
    int res = fs_unlink(current_full_path);
    if (res) {
        LOG_PRINTK("error deleting file in delete\n");
        return -1;
    }

    return 0;
}
// the nuclear option.
int clear_audio_directory()
{
    if (file_count == 1) {
        return 0;
    }
    // check if all files are zero
    //  char* path_ = "/SD:/audio";
    //  clear_audio_file(file_count);
    int res = 0;
    for (uint8_t i = file_count; i > 0; i--) {
        res = delete_audio_file(i);
        k_msleep(10);
        if (res) {
            LOG_PRINTK("error on %d\n", i);
            return -1;
        }
    }
    res = fs_unlink("/SD:/audio");
    if (res) {
        LOG_ERR("error deleting file");
        return -1;
    }
    res = fs_mkdir("/SD:/audio");
    if (res) {
        LOG_ERR("failed to make directory");
        return -1;
    }
    res = create_file("audio/a01.txt");
    if (res) {
        LOG_ERR("failed to make new file in directory files");
        return -1;
    }
    LOG_ERR("done with clearing");

    file_count = 1;
    move_write_pointer(1);
    return 0;
    // if files are cleared, then directory is oked for destrcution.
}

int save_offset(uint32_t offset)
{
    uint8_t buf[4] = {offset & 0xFF, (offset >> 8) & 0xFF, (offset >> 16) & 0xFF, (offset >> 24) & 0xFF};

    struct fs_file_t write_file;
    fs_file_t_init(&write_file);
    int res = fs_open(&write_file, "/SD:/info.txt", FS_O_WRITE | FS_O_CREATE);
    if (res) {
        LOG_ERR("error opening file %d", res);
        return -1;
    }
    res = fs_write(&write_file, &buf, 4);
    if (res < 0) {
        LOG_ERR("error writing file %d", res);
        return -1;
    }
    fs_close(&write_file);
    return 0;
}

int get_offset()
{
    uint8_t buf[4];
    struct fs_file_t read_file;
    fs_file_t_init(&read_file);
    int rc = fs_open(&read_file, "/SD:/info.txt", FS_O_READ | FS_O_RDWR);
    if (rc < 0) {
        LOG_ERR("error opening file %d", rc);
        return -1;
    }
    rc = fs_seek(&read_file, 0, FS_SEEK_SET);
    if (rc < 0) {
        LOG_ERR("error seeking file %d", rc);
        return -1;
    }
    rc = fs_read(&read_file, &buf, 4);
    if (rc < 0) {
        LOG_ERR("error reading file %d", rc);
        return -1;
    }
    fs_close(&read_file);
    uint32_t *offset_ptr = (uint32_t *) buf;
    LOG_INF("get offset is %d", offset_ptr[0]);
    fs_close(&read_file);

    return offset_ptr[0];
}

void sd_off()
{
    // Suspend SPI peripheral to save power
    const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
    if (device_is_ready(spi_dev)) {
        pm_device_action_run(spi_dev, PM_DEVICE_ACTION_SUSPEND);
    }
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), 15, GPIO_DISCONNECTED); // MOSI
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), 14, GPIO_DISCONNECTED); // MISO
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), 13, GPIO_DISCONNECTED); // SCK
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), 2, GPIO_DISCONNECTED);  // CS
    gpio_pin_set_dt(&sd_en_gpio_pin, 0);

    sd_enabled = false;
}

void sd_on()
{
    gpio_pin_set_dt(&sd_en_gpio_pin, 1);
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), 15, GPIO_OUTPUT);     // MOSI
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), 14, GPIO_INPUT);      // MISO
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio1)), 13, GPIO_OUTPUT);     // SCK
    gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), 2, GPIO_OUTPUT_HIGH); // CS
    const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
    if (device_is_ready(spi_dev)) {
        pm_device_action_run(spi_dev, PM_DEVICE_ACTION_RESUME);
    }
    sd_enabled = true;
}

bool is_sd_on()
{
    return sd_enabled;
}
