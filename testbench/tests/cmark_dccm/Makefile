
TEST := cmark_dccm
OFILES := crt0.o cmark_dccm.o printf.o 
BUILD_DIR := ../snapshots/default

all .DEFAULT :
	$(MAKE) -f $(RV_ROOT)/tools/MakeHex TEST=$(TEST) OFILES="$(OFILES)" \
		BUILD_DIR=$(BUILD_DIR) $@


