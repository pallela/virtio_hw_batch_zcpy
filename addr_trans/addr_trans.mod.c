#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xf027374b, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x7f966fb0, __VMLINUX_SYMBOL_STR(debugfs_remove) },
	{ 0x5e8ff345, __VMLINUX_SYMBOL_STR(debugfs_create_file) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xed5227ac, __VMLINUX_SYMBOL_STR(get_user_pages) },
	{ 0xa2930c8f, __VMLINUX_SYMBOL_STR(down_read) },
	{ 0x29952cd7, __VMLINUX_SYMBOL_STR(up_read) },
	{ 0xc6377d66, __VMLINUX_SYMBOL_STR(put_page) },
	{ 0xabe14130, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "6288512CFD22D0601C40301");
