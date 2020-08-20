load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "guide_message_manager",
    srcs = ["guide_message_manager.cc"],
    hdrs = ["guide_message_manager.h"],
    deps = [
        "//cyber",
	"//modules/drivers/canbus/can_comm:message_manager_base",
	"//modules/guide_can/protocol:canbus_guide_protocol",
	"//modules/guide_can/proto:chassis_detail_cc_proto",
    ],
)
cc_library(
    name = "guide_controller",
    srcs = [
        "guide_controller.cc",
    ],
    hdrs = [
        "guide_controller.h",
    ],
    deps = [
        ":guide_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/guide_can/proto:control_command_cc_proto",
        "//modules/guide_can/protocol:canbus_guide_protocol",
    ],
)


cpplint()
