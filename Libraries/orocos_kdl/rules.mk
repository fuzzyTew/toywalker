# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)/kdl

# Local flags
CXXFLAGS_$(d) := $(WIRISH_INCLUDES) $(LIBMAPLE_INCLUDES) -std=gnu++11 $(addprefix -I,$(USER_INCLUDES)) -Wno-deprecated-declarations -Wno-unused-local-typedefs -Wno-sign-compare

# Local rules and targets
cSRCS_$(d) :=

cppSRCS_$(d) := \
	kdl/chain.cpp \
	kdl/frames.cpp \
	kdl/joint.cpp \
	kdl/rigidbodyinertia.cpp \
	kdl/rotationalinertia.cpp \
	kdl/segment.cpp \
	kdl/tree.cpp \

cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)

OBJS_$(d) := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o) \
             $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
DEPS_$(d) := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CXXFLAGS := $(CXXFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include $(DEPS_$(d))
d := $(dirstack_$(sp))
sp := $(basename $(sp))
