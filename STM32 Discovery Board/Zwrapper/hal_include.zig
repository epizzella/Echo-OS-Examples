//All of your c imports now belong to us
pub const stm32 = @cImport({
    @cDefine("STM32F407xx", "");
    @cInclude("stm32f4xx_hal.h");
    @cInclude("gpio.h");
});

pub const HalStatus = struct {
    pub fn StatusToErr(status: stm32.HAL_StatusTypeDef) errors!void {
        switch (status) {
            1 => return errors.HAL_ERROR,
            2 => return errors.HAL_BUSY,
            3 => return errors.HAL_TIMEOUT,
            else => return,
        }
    }

    pub const errors = error{
        HAL_ERROR,
        HAL_BUSY,
        HAL_TIMEOUT,
    };
};
