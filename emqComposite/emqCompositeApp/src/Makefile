TOP=../..
include $(TOP)/configure/CONFIG

# The EMQ triplet's polarity sequence (README "emqComposite"): one SNL
# program, run once per magnet by `seq emqComposite, "..."` after iocInit.
LIBRARY_IOC = emqComposite
emqComposite_SRCS += emqComposite.st
emqComposite_LIBS += seq pv
emqComposite_LIBS += $(EPICS_BASE_IOC_LIBS)
DBD += emqComposite.dbd

include $(TOP)/configure/RULES
