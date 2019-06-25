ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SDM450) += sdm450-qrd-sku4-overlay.dtbo

sdm450-qrd-sku4-overlay.dtbo-base := sdm450-pmi632.dtb

else

dtb-$(CONFIG_ARCH_SDM450) += sdm450-rcm.dtb \
	sdm450-cdp.dtb \
	sdm450-mtp.dtb \
	sdm450-qrd.dtb \
	sdm450-pmi8940-mtp.dtb \
	sdm450-pmi8937-mtp.dtb \
	sdm450-iot-mtp.dtb \
	sdm450-qrd-sku4.dtb \
	sdm450-pmi632-cdp-s2.dtb \
	sdm450-pmi632-mtp-s3.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
