#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif


static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xaa6901ac, "__kfifo_out_r" },
	{ 0xc533ad8e, "devm_request_threaded_irq" },
	{ 0x45c954d9, "devm_kmalloc" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0x314b20c8, "scnprintf" },
	{ 0x5ce260ee, "sysfs_remove_groups" },
	{ 0xd7da531d, "sysfs_create_groups" },
	{ 0x9d669763, "memcpy" },
	{ 0x37a0cba, "kfree" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0xcaaf28e5, "gpiod_get_value" },
	{ 0x6f4f4b45, "devm_gpiod_get_optional" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x259aea13, "devm_gpiod_put" },
	{ 0x95d0e721, "of_get_property" },
	{ 0xb9db2671, "wake_up_process" },
	{ 0xd02c9bf9, "request_firmware_direct" },
	{ 0x23748d, "sysfs_notify" },
	{ 0x2ec524ad, "__kfifo_in_r" },
	{ 0x92997ed8, "_printk" },
	{ 0x3ea1b6e4, "__stack_chk_fail" },
	{ 0x7f505c6c, "devm_free_irq" },
	{ 0xbcf33737, "_dev_info" },
	{ 0xab713b03, "i2c_register_driver" },
	{ 0x2a5dcfb0, "_dev_err" },
	{ 0x20884306, "irq_get_irq_data" },
	{ 0xe8371cdf, "mutex_lock" },
	{ 0x84b183ae, "strncmp" },
	{ 0xa7345f8a, "kthread_stop" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0x8a01a639, "__mutex_init" },
	{ 0x5f754e5a, "memset" },
	{ 0x2a8a8e91, "_dev_warn" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0x45596827, "kthread_create_on_node" },
	{ 0x7e423ba3, "mutex_unlock" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
	{ 0xb43f9365, "ktime_get" },
	{ 0xeea479b, "i2c_transfer" },
	{ 0x5d3b3d94, "kmalloc_trace" },
	{ 0xdefadec8, "i2c_del_driver" },
	{ 0x37f614b7, "__kfifo_len_r" },
	{ 0x97255bdf, "strlen" },
	{ 0xd239511d, "gpiod_direction_output" },
	{ 0xceab0311, "strchrnul" },
	{ 0x2d6fcc06, "__kmalloc" },
	{ 0xbe866c8f, "kmalloc_caches" },
	{ 0xeff57adb, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Cams,tof8806");
MODULE_ALIAS("of:N*T*Cams,tof8806C*");
MODULE_ALIAS("i2c:tof8806");

MODULE_INFO(srcversion, "5E062E1F9CB12C1063842E3");
