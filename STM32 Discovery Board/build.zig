const std = @import("std");
const builtin = @import("builtin");

pub fn build(b: *std.Build) void {
    const prj_name = "DISCO_RTOS";

    comptime {
        const required_zig = "0.13.0";
        const current_zig = builtin.zig_version;
        const min_zig = std.SemanticVersion.parse(required_zig) catch unreachable;
        if (current_zig.order(min_zig) == .lt) {
            const error_message =
                \\Attempting to compile with an older version of zig. This project requires build {}
            ;
            @compileError(std.fmt.comptimePrint(error_message, .{min_zig}));
        }
    }

    //stm32f103
    const target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb,
        .cpu_model = std.zig.CrossTarget.CpuModel{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        .abi = .eabihf,
        .os_tag = .freestanding,
        .cpu_features_add = std.Target.arm.featureSet(&[_]std.Target.arm.Feature{std.Target.arm.Feature.fp_armv8d16sp}),
    });

    const output_dir = "Bin/";
    b.exe_dir = output_dir;

    const optimize = std.builtin.OptimizeMode.Debug;

    const elf = b.addExecutable(.{
        .name = prj_name ++ ".elf",
        .root_source_file = b.path("main.zig"),
        .target = target,
        .optimize = optimize,
        .single_threaded = true,
        .link_libc = false,
        .linkage = .static,
    });

    //Without setting the entry point a linker warning stating that _start or _exit is missing.
    //Since this is a freestanding binaray setting the entry point has no effect on functoinality.
    // elf.entry = .{ .symbol_name = "Reset_Handler" };
    elf.entry = .disabled;

    //#defines for STM32 HAL
    elf.defineCMacro("USE_HAL_DRIVER", "");
    elf.defineCMacro("STM32F407xx", "");

    elf.addAssemblyFile(.{ .src_path = .{ .owner = b, .sub_path = "startup_stm32f407xx.s" } });
    elf.setLinkerScript(.{ .src_path = .{ .owner = b, .sub_path = "STM32F407VGTx_FLASH.ld" } });
    //elf.setVerboseLink(true);

    const c_flags = [_][]const u8{
        "-Og",
        "-g3", //max debug symbols
        "-Wall", //This enables all the warnings about constructions that some users consider questionable, and that are easy to avoid, even in conjunction with macros.
        "-Wextra", //This enables some extra warning flags that are not enabled by -Wall.
        "-mthumb", //Requests that the compiler targets the thumb instruction set.
        "-mlittle-endian", //arm is little endian
    };

    const stm32f4_hal_src = [_][]const u8{
        "Core/Src/main.c",
        "Core/Src/gpio.c",
        "Core/Src/i2c.c",
        "Core/Src/i2s.c",
        "Core/Src/spi.c",
        "Core/Src/stm32f4xx_it.c",
        "Core/Src/stm32f4xx_hal_msp.c",
        "Core/Src/system_stm32f4xx.c",
        "Core/Src/sysmem.c",
        "Core/Src/syscalls.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c",
    };

    const stm32f4_hal_inc = [_][]const u8{
        "Core/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
        "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
        "Drivers/CMSIS/Include",
    };

    // Try to find arm-none-eabi-gcc program at a user specified path, or PATH variable if none provided
    const arm_gcc_pgm = if (b.option([]const u8, "armgcc", "Path to arm-none-eabi-gcc compiler")) |arm_gcc_path|
        b.findProgram(&.{"arm-none-eabi-gcc"}, &.{arm_gcc_path}) catch {
            std.log.err("Couldn't find arm-none-eabi-gcc at provided path: {s}\n", .{arm_gcc_path});
            unreachable;
        }
    else
        b.findProgram(&.{"arm-none-eabi-gcc"}, &.{}) catch {
            std.log.err("Couldn't find arm-none-eabi-gcc in PATH, try manually providing the path to this executable with -Darmgcc=[path]\n", .{});
            unreachable;
        };

    // Allow user to enable float formatting in newlib (printf, sprintf, ...)
    if (b.option(bool, "NEWLIB_PRINTF_FLOAT", "Force newlib to include float support for printf()")) |_| {
        elf.forceUndefinedSymbol("_printf_float"); // GCC equivalent : "-u _printf_float"
    }

    //  Use gcc-arm-none-eabi to figure out where library paths are
    const gcc_arm_sysroot_path = std.mem.trim(u8, b.run(&.{ arm_gcc_pgm, "-print-sysroot" }), "\r\n");
    const gcc_arm_multidir_relative_path = std.mem.trim(u8, b.run(&.{ arm_gcc_pgm, "-mcpu=cortex-m4", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard", "-print-multi-directory" }), "\r\n");
    const gcc_arm_version = std.mem.trim(u8, b.run(&.{ arm_gcc_pgm, "-dumpversion" }), "\r\n");
    const gcc_arm_lib_path1 = b.fmt("{s}/../lib/gcc/arm-none-eabi/{s}/{s}", .{ gcc_arm_sysroot_path, gcc_arm_version, gcc_arm_multidir_relative_path });
    const gcc_arm_lib_path2 = b.fmt("{s}/lib/{s}", .{ gcc_arm_sysroot_path, gcc_arm_multidir_relative_path });

    // Manually add "nano" variant newlib C standard lib from arm-none-eabi-gcc library folders
    elf.addLibraryPath(.{ .cwd_relative = gcc_arm_lib_path1 });
    elf.addLibraryPath(.{ .cwd_relative = gcc_arm_lib_path2 });
    elf.addSystemIncludePath(.{ .cwd_relative = b.fmt("{s}/include", .{gcc_arm_sysroot_path}) });
    elf.linkSystemLibrary("c_nano");
    elf.linkSystemLibrary("m");

    // Manually include C runtime objects bundled with arm-none-eabi-gcc
    elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crt0.o", .{gcc_arm_lib_path2}) });
    elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crti.o", .{gcc_arm_lib_path1}) });
    elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crtbegin.o", .{gcc_arm_lib_path1}) });
    elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crtend.o", .{gcc_arm_lib_path1}) });
    elf.addObjectFile(.{ .cwd_relative = b.fmt("{s}/crtn.o", .{gcc_arm_lib_path1}) });

    //elf.want_lto = true; //silence ld.lld tripples warning... doesn't work
    elf.link_gc_sections = true; //equivalent to -Wl,--gc-sections
    elf.link_data_sections = true;
    elf.link_function_sections = true;

    //Add c source files
    elf.addCSourceFiles(.{
        .files = &stm32f4_hal_src,
        .flags = &c_flags,
    });

    //Add c inc paths
    for (stm32f4_hal_inc) |header| {
        elf.addIncludePath(.{ .src_path = .{ .owner = b, .sub_path = header } });
    }

    //Add RTOS package
    const rtos = b.dependency("EchoOS", .{ .target = target, .optimize = optimize });
    elf.root_module.addImport("EchoOS", rtos.module("EchoOS"));

    //build the elf file
    b.installArtifact(elf);

    const objcpy_bin = elf.addObjCopy(.{ .format = .bin });
    const bin_generate = b.addInstallBinFile(objcpy_bin.getOutput(), prj_name ++ ".bin");

    objcpy_bin.step.dependOn(&elf.step);
    bin_generate.step.dependOn(&objcpy_bin.step);
    b.default_step.dependOn(&bin_generate.step);

    const objcpy_hex = elf.addObjCopy(.{ .format = .hex });
    const hex_generate = b.addInstallBinFile(objcpy_hex.getOutput(), prj_name ++ ".hex");

    objcpy_hex.step.dependOn(&elf.step);
    bin_generate.step.dependOn(&objcpy_hex.step);
    b.default_step.dependOn(&hex_generate.step);

    //Additional Steps:
    //Clean workspace
    const clean_step = b.step("clean", "Cleans the workspace");
    clean_step.dependOn(&b.addRemoveDirTree(b.pathFromRoot(output_dir)).step);
    clean_step.dependOn(&b.addRemoveDirTree(b.pathFromRoot("zig-cache")).step);

    //Flash the mcu
    const openocd_flash_cmd = b.addSystemCommand(&.{
        "openocd", //openocd must be in path
        "-f", "interface/stlink.cfg", //config for stlink.  stlink must be isntalled.
        "-f", "target/stm32f1x.cfg", //config for target mcu
        "-c", "program " ++ output_dir ++ prj_name ++ ".elf " ++ "verify reset exit", //program the elf file to board
    });

    const flash_step = b.step("flash", "Runs Openocd to flash the mcu.");
    flash_step.dependOn(&openocd_flash_cmd.step);
}
