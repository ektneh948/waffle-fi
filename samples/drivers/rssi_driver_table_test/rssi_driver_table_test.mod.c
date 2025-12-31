#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
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
	{ 0x9d3bc545, "module_layout" },
	{ 0x52b3c7de, "class_unregister" },
	{ 0xdbb05a69, "device_destroy" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xd1c14abb, "class_destroy" },
	{ 0x6033c666, "device_create" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0xda47c7f0, "__class_create" },
	{ 0x2cc5bbe2, "__register_chrdev" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x8da6585d, "__stack_chk_fail" },
	{ 0x4829a47e, "memcpy" },
	{ 0xa916b694, "strnlen" },
	{ 0xdcb764ad, "memset" },
	{ 0x92997ed8, "_printk" },
	{ 0xe2d5255a, "strcmp" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0x1e6d26a8, "strstr" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x1e49e99f, "cpu_hwcaps" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x31909811, "cpu_hwcap_keys" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0x37a0cba, "kfree" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x96848186, "scnprintf" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x61bf0d12, "kmem_cache_alloc_trace" },
	{ 0xa0010230, "kmalloc_caches" },
	{ 0x4b0a3f52, "gic_nonsecure_priorities" },
	{ 0xec3d2e1b, "trace_hardirqs_off" },
	{ 0xd697e69a, "trace_hardirqs_on" },
	{ 0xb788fb30, "gic_pmr_sync" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "908A1ECB336405839F535E5");
