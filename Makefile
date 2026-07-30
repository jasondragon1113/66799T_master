################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

################################################################################
# PID 調參版（另一支遙控程式，不是模式開關）
#
#   pros make tune                     -> 整包清乾淨、帶 -DPID_TUNE_PROGRAM 重編
#   pros upload --slot 2 --name "66799T TUNE"
#
# 正常比賽版照舊，什麼都不用加：pros mu --slot 1
#
# clean 不是可有可無的：這次唯一的差別只有一個 -D 旗標，make 從檔案時間戳看不出
# 任何檔案「變舊了」，不清乾淨的話會拿到一半調參版一半正常版的物件檔。
#
# The tuning program is a compile-time variant, not a runtime mode: with
# PID_TUNE_PROGRAM defined, opcontrol() dispatches to tune_opcontrol()
# (src/tune_opcontrol.cpp) and the normal driving loop is not compiled at all.
# The clean is mandatory -- only a -D flag changes, which make cannot see in the
# timestamps.
################################################################################
.PHONY: tune
tune:
	$(MAKE) clean
	$(MAKE) quick EXTRA_CXXFLAGS="-DPID_TUNE_PROGRAM"
