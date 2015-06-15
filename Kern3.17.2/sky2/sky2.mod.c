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
	{ 0x6ac05b47, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x6e327bf7, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0x1028ccca, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0xb5292d21, __VMLINUX_SYMBOL_STR(pci_set_power_state) },
	{ 0x8387f321, __VMLINUX_SYMBOL_STR(pci_wake_from_d3) },
	{ 0x88bfa7e, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0xd5f2172f, __VMLINUX_SYMBOL_STR(del_timer_sync) },
	{ 0xbe7f94fc, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0x33201828, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0xedc03953, __VMLINUX_SYMBOL_STR(iounmap) },
	{ 0x4379595d, __VMLINUX_SYMBOL_STR(free_netdev) },
	{ 0x5465db08, __VMLINUX_SYMBOL_STR(unregister_netdev) },
	{ 0x593a99b, __VMLINUX_SYMBOL_STR(init_timer_key) },
	{ 0xbc9b4113, __VMLINUX_SYMBOL_STR(register_netdev) },
	{ 0x97b40490, __VMLINUX_SYMBOL_STR(netif_napi_add) },
	{ 0x213e79e5, __VMLINUX_SYMBOL_STR(pci_disable_msi) },
	{ 0x42c8de35, __VMLINUX_SYMBOL_STR(ioremap_nocache) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x754d539c, __VMLINUX_SYMBOL_STR(strlen) },
	{ 0x66a52190, __VMLINUX_SYMBOL_STR(pci_set_master) },
	{ 0x17208528, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0x2ebc9369, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x9166fada, __VMLINUX_SYMBOL_STR(strncpy) },
	{ 0xfa66f77c, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x34f22f94, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0xf432dd3d, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0xcf21d241, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0x28373a93, __VMLINUX_SYMBOL_STR(alloc_etherdev_mqs) },
	{ 0xc94fbfb4, __VMLINUX_SYMBOL_STR(eth_validate_addr) },
	{ 0xe5915154, __VMLINUX_SYMBOL_STR(ethtool_op_get_link) },
	{ 0xfaf98462, __VMLINUX_SYMBOL_STR(bitrev32) },
	{ 0x393d4de9, __VMLINUX_SYMBOL_STR(crc32_le) },
	{ 0x5792f848, __VMLINUX_SYMBOL_STR(strlcpy) },
	{ 0xd7afb5c9, __VMLINUX_SYMBOL_STR(device_set_wakeup_enable) },
	{ 0x6e720ff2, __VMLINUX_SYMBOL_STR(rtnl_unlock) },
	{ 0xc7a4fbed, __VMLINUX_SYMBOL_STR(rtnl_lock) },
	{ 0x30e08322, __VMLINUX_SYMBOL_STR(netif_device_attach) },
	{ 0x6f723f0c, __VMLINUX_SYMBOL_STR(netif_device_detach) },
	{ 0xc89ad6a8, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0x867f5663, __VMLINUX_SYMBOL_STR(napi_complete) },
	{ 0x91eb9b4, __VMLINUX_SYMBOL_STR(round_jiffies) },
	{ 0xaeed6734, __VMLINUX_SYMBOL_STR(eth_type_trans) },
	{ 0x898ccdb4, __VMLINUX_SYMBOL_STR(dev_notice) },
	{ 0x19d343a6, __VMLINUX_SYMBOL_STR(napi_gro_receive) },
	{ 0x817a853e, __VMLINUX_SYMBOL_STR(netif_receive_skb) },
	{ 0x5b2d081c, __VMLINUX_SYMBOL_STR(skb_put) },
	{ 0xf0b0ca43, __VMLINUX_SYMBOL_STR(dev_close) },
	{ 0xe74bceb9, __VMLINUX_SYMBOL_STR(netdev_err) },
	{ 0x8834396c, __VMLINUX_SYMBOL_STR(mod_timer) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0xf720e9ee, __VMLINUX_SYMBOL_STR(netif_carrier_on) },
	{ 0xe523ad75, __VMLINUX_SYMBOL_STR(synchronize_irq) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0x7711e70d, __VMLINUX_SYMBOL_STR(netdev_printk) },
	{ 0x71b9ad6, __VMLINUX_SYMBOL_STR(netdev_info) },
	{ 0x8c2aa244, __VMLINUX_SYMBOL_STR(netif_carrier_off) },
	{ 0x47d25521, __VMLINUX_SYMBOL_STR(netdev_update_features) },
	{ 0xc5f21385, __VMLINUX_SYMBOL_STR(pci_find_capability) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x6f5811c6, __VMLINUX_SYMBOL_STR(consume_skb) },
	{ 0x35b1ce39, __VMLINUX_SYMBOL_STR(kfree_skb) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xe132ded2, __VMLINUX_SYMBOL_STR(netdev_warn) },
	{ 0xf6ebc03b, __VMLINUX_SYMBOL_STR(net_ratelimit) },
	{ 0x18ba993a, __VMLINUX_SYMBOL_STR(pci_pme_capable) },
	{ 0x877d1a23, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x4563540b, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0xb6b46a7c, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x67f1e865, __VMLINUX_SYMBOL_STR(dma_set_mask) },
	{ 0x6582055f, __VMLINUX_SYMBOL_STR(pci_enable_msi_range) },
	{ 0x3cb91517, __VMLINUX_SYMBOL_STR(pci_bus_write_config_dword) },
	{ 0x6783122c, __VMLINUX_SYMBOL_STR(pci_bus_read_config_dword) },
	{ 0x7a2af7b4, __VMLINUX_SYMBOL_STR(cpu_number) },
	{ 0x6d6494fc, __VMLINUX_SYMBOL_STR(__dev_kfree_skb_any) },
	{ 0x4ea25709, __VMLINUX_SYMBOL_STR(dql_reset) },
	{ 0xa00aca2a, __VMLINUX_SYMBOL_STR(dql_completed) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x7e5ab90e, __VMLINUX_SYMBOL_STR(__netif_schedule) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0xa1c76e0a, __VMLINUX_SYMBOL_STR(_cond_resched) },
	{ 0xab4c4e25, __VMLINUX_SYMBOL_STR(__napi_schedule) },
	{ 0x2072ee9b, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0x179b87b6, __VMLINUX_SYMBOL_STR(put_page) },
	{ 0xb7eabe5b, __VMLINUX_SYMBOL_STR(__netdev_alloc_skb) },
	{ 0x4d33086, __VMLINUX_SYMBOL_STR(dma_supported) },
	{ 0x16305289, __VMLINUX_SYMBOL_STR(warn_slowpath_null) },
	{ 0x7bcbf885, __VMLINUX_SYMBOL_STR(x86_dma_fallback_dev) },
	{ 0x888bbb98, __VMLINUX_SYMBOL_STR(dma_ops) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0xe285b2c1, __VMLINUX_SYMBOL_STR(alloc_pages_current) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0xba63339c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_bh) },
	{ 0x1637ff0f, __VMLINUX_SYMBOL_STR(_raw_spin_lock_bh) },
	{ 0xd52bf1ce, __VMLINUX_SYMBOL_STR(_raw_spin_lock) },
	{ 0x3c3fce39, __VMLINUX_SYMBOL_STR(__local_bh_enable_ip) },
	{ 0x4629334c, __VMLINUX_SYMBOL_STR(__preempt_count) },
	{ 0x4c9d28b0, __VMLINUX_SYMBOL_STR(phys_base) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v00001148d00009000sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001148d00009E00sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001148d00009E01sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001186d00004B00sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001186d00004001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001186d00004B02sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001186d00004B03sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004340sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004341sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004342sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004343sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004344sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004345sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004346sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004347sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004350sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004351sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004352sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004353sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004354sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004355sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004356sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004357sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd0000435Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004360sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004361sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004362sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004363sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004364sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004365sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004366sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004367sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004368sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004369sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd0000436Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd0000436Bsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd0000436Csv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd0000436Dsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004370sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004380sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004381sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000011ABd00004382sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "0C859CF393F1D2ABE9F86D2");
