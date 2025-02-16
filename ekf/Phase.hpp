#ifndef PHASE_HPP_
#define PHASE_HPP_

#include <string>

class Phase
{
public:
    uint32_t start_time;
    void set_start_time(uint32_t new_start_time)
    {
        start_time = new_start_time;
    }
    virtual uint8_t get_id();
};

class Initialization : public Phase
{
    uint8_t get_id()
    {
        return 0;
    }
};

class Stabilization : public Phase
{
    uint8_t get_id()
    {
        return 1;
    }
};

class Standby : public Phase
{
    uint8_t get_id()
    {
        return 2;
    }
};

class Deployment : public Phase
{
    uint8_t get_id()
    {
        return 3;
    }
};

class Armed : public Phase
{
    uint8_t get_id()
    {
        return 4;
    }
};

class InSun : public Phase
{
    uint8_t get_id()
    {
        return 5;
    }
};

class Firing : public Phase
{
    uint8_t get_id()
    {
        return 6;
    }
};

#endif