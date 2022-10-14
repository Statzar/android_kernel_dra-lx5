

#ifndef __BTRFS_TESTS
#define __BTRFS_TESTS

#ifdef CONFIG_BTRFS_FS_RUN_SANITY_TESTS

#define test_msg(fmt, ...) pr_info("BTRFS: selftest: " fmt, ##__VA_ARGS__)

struct btrfs_root;

int btrfs_test_free_space_cache(void);
int btrfs_test_extent_buffer_operations(void);
int btrfs_test_extent_io(void);
int btrfs_test_inodes(void);
int btrfs_test_qgroups(void);
int btrfs_init_test_fs(void);
void btrfs_destroy_test_fs(void);
struct inode *btrfs_new_test_inode(void);
struct btrfs_fs_info *btrfs_alloc_dummy_fs_info(void);
void btrfs_free_dummy_root(struct btrfs_root *root);
#else
static inline int btrfs_test_free_space_cache(void)
{
	return 0;
}
static inline int btrfs_test_extent_buffer_operations(void)
{
	return 0;
}
static inline int btrfs_init_test_fs(void)
{
	return 0;
}
static inline void btrfs_destroy_test_fs(void)
{
}
static inline int btrfs_test_extent_io(void)
{
	return 0;
}
static inline int btrfs_test_inodes(void)
{
	return 0;
}
static inline int btrfs_test_qgroups(void)
{
	return 0;
}
#endif

#endif
