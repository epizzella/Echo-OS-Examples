const hal = @import("Zwrapper/hal_include.zig").stm32;
const zgpio = @import("Zwrapper/gpio_wrapper.zig").Zgpio;
const Os = @import("EchoOS");

//Note:  zigMain is call from main.c
export fn zigMain() void {
    // Initalize the tasks
    mainTask.init();
    task2.init();
    task3.init();
    task4.init();

    sem.init() catch {};
    event_group.init() catch {};
    msg_queue.init() catch {};

    // Begin multitasking
    Os.startOS(.{
        .clock_config = .{
            .cpu_clock_freq_hz = 180_000_000,
            .os_sys_clock_freq_hz = 1_000,
        },
        .os_tick_callback = &incTick,
        .timer_config = .{
            .timer_enable = true,
            .timer_stack_size = stackSize,
            .timer_task_priority = 4,
        },
    });

    unreachable;
}

fn mainTaskSubroutine() !void {
    // Create a timer to toggle an led
    var timer = Os.Timer.create(.{ .name = "timer", .callback = &timerCallback });
    try timer.set(.{ .autoreload = true, .timeout_ms = 250 });
    try timer.start();

    while (true) {
        try Os.Time.delay(100);
        try sem.post(.{}); //increment the semaphore

        try Os.Time.delay(100);
        try event_group.writeEvent(.{ .event = 1 }); //Write an event

        try Os.Time.delay(100);
        try msg_queue.pushMsg(.toggle);
    }
}

fn task2Subroutine() !void {
    while (true) {
        // Wait on the sempahore.  Default options used therefore semaphore will never timeout.
        try sem.wait(.{});
        led1.TogglePin();
    }
}

fn task3Subroutine() !void {
    while (true) {
        // Wait for bit 1 to be set
        const event = try event_group.awaitEvent(.{ .event_mask = 1, .PendOn = .any_set });
        led2.TogglePin();
        // Clear the event
        try event_group.writeEvent(.{ .event = ~event });
    }
}

fn task4Subroutine() !void {
    while (true) {
        // Wait for a message to enter the queue
        const msg = try msg_queue.awaitMsg(.{});

        if (msg == LedMsg.toggle) {
            led3.TogglePin();
        }
    }
}

fn timerCallback() void {
    led4.TogglePin();
}

extern var uwTick: c_uint;
fn incTick() void {
    uwTick += 1;
}

//Create the synchronization objects.
var sem = Os.Semaphore.create_semaphore(.{ .name = "led_semaphore", .inital_value = 2 });
var event_group = Os.EventGroup.createEventGroup(.{ .name = "led_event_group" });
const MsgQueue = Os.createMsgQueueType(.{ .buffer_size = 5, .MsgType = LedMsg });
var msg_queue = MsgQueue.createQueue(.{ .name = "led_msg", .inital_val = LedMsg.off });
const LedMsg = enum { toggle, on, off };

var led1: zgpio = .{ .m_port = hal.GPIOD, .m_pin = hal.GPIO_PIN_12 };
var led2: zgpio = .{ .m_port = hal.GPIOD, .m_pin = hal.GPIO_PIN_13 };
var led3: zgpio = .{ .m_port = hal.GPIOD, .m_pin = hal.GPIO_PIN_14 };
var led4: zgpio = .{ .m_port = hal.GPIOD, .m_pin = hal.GPIO_PIN_15 };

const stackSize = 200;
var stack1: [stackSize]u32 = [_]u32{0xDEADC0DE} ** stackSize;
var stack2: [stackSize]u32 = [_]u32{0xDEADC0DE} ** stackSize;
var stack3: [stackSize]u32 = [_]u32{0xDEADC0DE} ** stackSize;
var stack4: [stackSize]u32 = [_]u32{0xDEADC0DE} ** stackSize;

var mainTask = Os.Task.create_task(.{
    .name = "main task",
    .priority = 0,
    .stack = &stack1,
    .subroutine = &mainTaskSubroutine,
});

var task2 = Os.Task.create_task(.{
    .name = "task2",
    .priority = 1,
    .stack = &stack2,
    .subroutine = &task2Subroutine,
});

var task3 = Os.Task.create_task(.{
    .name = "task3",
    .priority = 2,
    .stack = &stack3,
    .subroutine = &task3Subroutine,
});

var task4 = Os.Task.create_task(.{
    .name = "task4",
    .priority = 3,
    .stack = &stack4,
    .subroutine = &task4Subroutine,
});
