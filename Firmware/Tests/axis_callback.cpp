

#include <doctest.h>
#include <algorithm>
#include <cstring>
class Axis  {
public:

    static constexpr int PARAM_LEN = 256;
    unsigned ext_cfg[PARAM_LEN] = {0};

    enum EXT_CONFIG_REG
    {
        EXT_CONFIG_REG_ENABLE_NOTCH_FILTER = 0,
        EXT_CONFIG_REG_ENABLE_DC_BUS_OVER_VOLTAGE_FILTER = 1,
        EXT_CONFIG_REG_ENABLE_DC_BUS_UNDER_VOLTAGE_FILTER = 2,
        EXT_CONFIG_REG_ENABLE_OVER_TEMP_FILTER = 3,
        EXT_CONFIG_REG_ENABLE_CURRENT_LIMIT_VIOLATION_FILTER = 4,
        EXT_CONFIG_REG_KP_GAIN= 5,
        EXT_CONFIG_REG_KD_GAIN= 6,

    };

    typedef void (*ext_config_reg_callback_fun)(class Axis* axis, const unsigned int reg_value);
 


    typedef struct 
    {
        enum EXT_CONFIG_REG reg;
        ext_config_reg_callback_fun callback;

    }ext_config_reg_callback_fun_t;

    ext_config_reg_callback_fun_t ext_config_reg_callback_fun_[PARAM_LEN] = {
        {EXT_CONFIG_REG_ENABLE_NOTCH_FILTER, nullptr},
        {EXT_CONFIG_REG_ENABLE_DC_BUS_OVER_VOLTAGE_FILTER, nullptr},
        {EXT_CONFIG_REG_ENABLE_DC_BUS_UNDER_VOLTAGE_FILTER, nullptr},
        {EXT_CONFIG_REG_ENABLE_OVER_TEMP_FILTER, nullptr},
        {EXT_CONFIG_REG_ENABLE_CURRENT_LIMIT_VIOLATION_FILTER, nullptr},
        {EXT_CONFIG_REG_KP_GAIN, [](class Axis* axis, const unsigned int value) { axis->kp_gain_ = value; }},
        {EXT_CONFIG_REG_KD_GAIN, [](class Axis* axis, const unsigned int value) { axis->kd_gain_ = value; }}
    };


    float kp_gain_ = 3.0f;
    float kd_gain_ = 3.0f;
    bool set_ext_config(const unsigned int reg, const unsigned int value) {
        if (reg < PARAM_LEN) {
            ext_cfg[reg] = value;
            // 使用传统的索引遍历
            for(int i = 0; i < sizeof(ext_config_reg_callback_fun_)/sizeof(ext_config_reg_callback_fun_t); ++i) {
                if(ext_config_reg_callback_fun_[i].reg == reg && 
                ext_config_reg_callback_fun_[i].callback != nullptr) {
                    ext_config_reg_callback_fun_[i].callback(this,value);
                    break;
                }
            }
            return true;
        }
        return false;
    };
};


#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

TEST_CASE("Testing Axis Configuration") {
    Axis axis;

    SUBCASE("Initial values") {
        CHECK(axis.kp_gain_ == doctest::Approx(3.0f));
        CHECK(axis.kd_gain_ == doctest::Approx(3.0f));
        
        for (int i = 0; i < Axis::PARAM_LEN; i++) {
            CHECK(axis.ext_cfg[i] == 0);
        }
    }

    SUBCASE("Set config values") {
        CHECK(axis.set_ext_config(Axis::EXT_CONFIG_REG_KP_GAIN, 10));
        CHECK(axis.kp_gain_ == doctest::Approx(10.0f));
        CHECK(axis.ext_cfg[Axis::EXT_CONFIG_REG_KP_GAIN] == 10);

        CHECK(axis.set_ext_config(Axis::EXT_CONFIG_REG_KD_GAIN, 20));
        CHECK(axis.kd_gain_ == doctest::Approx(20.0f));
        CHECK(axis.ext_cfg[Axis::EXT_CONFIG_REG_KD_GAIN] == 20);
    }

    SUBCASE("Null callback registers") {
        CHECK(axis.set_ext_config(Axis::EXT_CONFIG_REG_ENABLE_NOTCH_FILTER, 1));
        CHECK(axis.ext_cfg[Axis::EXT_CONFIG_REG_ENABLE_NOTCH_FILTER] == 1);
    }

    SUBCASE("Invalid registers") {
        CHECK_FALSE(axis.set_ext_config(Axis::PARAM_LEN + 1, 100));
        CHECK_FALSE(axis.set_ext_config(-1, 1));
    }

    SUBCASE("Register boundaries") {
        CHECK(axis.set_ext_config(0, 1));
        CHECK(axis.set_ext_config(Axis::PARAM_LEN - 1, 1));
    }
}
