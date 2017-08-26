
#include "FreeRTOS.h"
#include "analysis.h"
#include "app.h"


#define BUFFER_SIZE         5


static uint32_t heartbeat_none_cnt = 0, breath_none_cnt = 0;
uint16_t heartbeat_buf[BUFFER_SIZE] = {0}, breath_buf[BUFFER_SIZE] = {0};
volatile uint8_t heartbeat_buf_idx = 0, breath_buf_idx = 0;
int32_t heartbeat_diff, heartbeat_diff_sum = 0;
int32_t breath_diff, breath_diff_sum = 0;
uint16_t breath_diff_average;


uint8_t heartbeat_cnt = 0;
uint8_t breath_cnt = 0;
uint8_t turning_cnt = 0;

uint8_t heartbeat_rate = 0;
uint8_t breath_rate = 0;
uint8_t turning_flag = 0;

uint32_t heart_rate_analysis(uint16_t x, uint32_t time)
{
    static uint8_t first_data_flag = 1;
    static uint16_t maxV, r_min, l_min;
    static uint32_t r_min_idx, l_min_idx, maxV_idx, pre_maxV_idx;
    //find_max_flag = 0;// 0 for initial state, 1 for found the max value
    static uint8_t find_r_min_flag;
    static uint8_t r_min_cnt;

    static uint8_t find_downhill_flag;
    static uint16_t tmp_max, tmp_min;
    static uint8_t tmp_min_flag, tmp_min_cnt;
    static uint32_t tmp_min_idx;

    if (first_data_flag == 1)
    {
        first_data_flag = 0;
        maxV = x, r_min = x, l_min = x; tmp_max = x, tmp_min = x;
        r_min_idx = time, l_min_idx = time, maxV_idx = time, pre_maxV_idx = time;

        find_r_min_flag = 0; r_min_cnt = 0;
        tmp_min_flag = 0; tmp_min_cnt = 0;
        find_downhill_flag = 1;

        return 0;
    }

    if (find_downhill_flag == 1)  //find the downtrend and get the minimum value
    {
        if (x > tmp_max)
        {
            tmp_max = x;
        }
        else if (tmp_min_flag == 0)  // first point in downhill
        {
            tmp_min = x; tmp_min_flag = 1;
        }
        else
        {
            if (x <= tmp_min) // store the latest minimum value
            {
                tmp_min = x; tmp_min_idx = time; tmp_min_cnt = 0;
            }
            else  // there is some value that bigger than tmp_min
            {
                tmp_min_cnt = tmp_min_cnt + 1;
                if ((tmp_min_cnt >= 3) || (x > tmp_min + 3))
                {
                    l_min = tmp_min; l_min_idx = tmp_min_idx;
                    maxV = x; maxV_idx = time;   // prepare to find the maximum value
                    find_downhill_flag = 0;
                    tmp_min_flag = 0;
                }
            }
        }
        return 0;  // return and wait for next point untill find the bottom on the left side
    }

    // following code is used to find the maximum value and bottom on the right side
    if ((x > maxV) && (find_r_min_flag == 0))  // find new high point
    {
        maxV = x; maxV_idx = time; find_r_min_flag = 0;
    }
    else if (find_r_min_flag == 0)  // record as the first r_min
    {
        r_min = x; r_min_idx = time; find_r_min_flag = 1;
    }
    else if (x <= r_min)
    {
        r_min = x; r_min_idx = time; r_min_cnt = 0;
    }
    else // x > r_min
    {
        r_min_cnt = r_min_cnt + 1;
        if ((r_min_cnt >= 3) || (x > r_min + 5))
        {
            uint16_t diff = r_min > l_min ? (r_min - l_min) : (l_min - r_min);
            uint16_t smaller = (maxV-r_min) < (maxV-l_min) ? (maxV-r_min) : (maxV-l_min);
            if ((r_min_idx - l_min_idx <= 18) && (diff < smaller * 0.8)
                && (maxV - l_min > 50) && (maxV_idx - pre_maxV_idx > 20))
            {
                pre_maxV_idx = maxV_idx;
                //heart_rate(maxV_idx) = 100;
                find_downhill_flag = 1;  // need to find another left bottom of the next heartbeat
                find_r_min_flag = 0;
                tmp_max = x; tmp_min = x;
                return maxV_idx;  // analysis one heartbeat point successfully, return the time
            }
            else
            {
                l_min = r_min; l_min_idx = r_min_idx;// record the r_min as l_min
                maxV = x; maxV_idx = time; find_r_min_flag = 0;
            }
        }
        else
        {
        }
    }
    return 0;
}

uint16_t filter_data;
uint8_t breath_flag = 0;
uint32_t breath_rate_analysis(uint16_t x, uint32_t time)
{
    static uint8_t data_initial_flag = 1;
    float alpha = 0.1;
    static uint16_t u = 0, u_pre;
    static uint16_t u_max, u_min;
    static uint32_t u_max_idx, u_min_idx;
    static uint8_t u_min_cnt, u_max_change_flag, u_win_cnt;
    uint32_t breath_idx = 0;

    if (data_initial_flag == 1)
    {
        data_initial_flag = 0;
        u_pre = x; u_max = x, u_min = x;
        u_max_idx = time; u_min_idx = time;
        u_min_cnt = 0; u_max_change_flag = 0; u_win_cnt = 0;

        return 0;
    }


    u = alpha * x + (1 - alpha) * u_pre;
    filter_data = u;
    u_pre = u;
    if (u > u_max)
    {
        u_max = u; u_max_idx = time;
        u_max_change_flag = 1;
    }
    else if (u < u_min)
    {
        u_min = u; u_min_idx = time;
        u_min_cnt = 0;
        if (u_max_change_flag == 1)
            u_max_change_flag = 2;
    }
    else
    {
        u_min_cnt = u_min_cnt + 1; // count the minimum value sustain for time
        if (u_max_change_flag == 1)
            u_max_change_flag = 2;
    }

    if ((u_min_cnt > 10) && (u_max_change_flag == 0)) // no other value bigger than initial u_max
    {
        u_max_change_flag = 1;
        u_max = u; u_max_idx = time;
    }

    if (u_max_change_flag == 2) // find the minimum value (u > u_min) and sustain for some time
        u_win_cnt = u_win_cnt + 1;
    else
        u_win_cnt = 0;

    if (u_win_cnt == 40) //find the new minmum value on the right side
    {
        if (u > u_min)
        {
            u_min = u; u_min_idx = time;
        }
    }
    if (u_win_cnt > 70) // u_max is the maximum value in past 70 value
    {
        if (u_max - u_min > 8)        //10
        {
            //breath_rate(u_max_idx) = 150;
            breath_idx = u_max_idx;
            u_max = u_min; u_max_idx = u_min_idx;
            u_win_cnt = 0; u_max_change_flag = 0;

            if (breath_flag)
                breath_flag = 0;
            else
                breath_flag = 50;
        }
    }
    return breath_idx;
}

#define X_MAX   255
uint32_t turning_analysis(uint16_t x, uint32_t time)
{
    static uint8_t find_turning_flag = 0;

    if (x >= X_MAX)
        find_turning_flag = 1;
    else if (find_turning_flag == 1)
    {
        find_turning_flag = 0;
        //turning(i - 1) = 170;
        return time - 1;
    }
    return 0;
}

void sleep_time_analysis(uint16_t x, uint32_t time_idx)
{
    static int32_t heartbeat_time = 0, heartbeat_time_pre = 0, heartbeat_cnt = 0;
    static int32_t breath_time = 0, breath_time_pre = 0, breath_cnt = 0;
    static uint8_t turning_tmp = 0;

    heartbeat_time = heart_rate_analysis(x, time_idx);
    if (heartbeat_time > 0)
    {
        heartbeat_cnt++;
        heartbeat_diff = heartbeat_time - heartbeat_time_pre;
        if ((heartbeat_diff < 100) && (heartbeat_diff > 25))// 30 < rate < 120
        {
            heartbeat_diff_sum -= heartbeat_buf[heartbeat_buf_idx];
            heartbeat_diff_sum += heartbeat_diff;
            heartbeat_buf[heartbeat_buf_idx] = heartbeat_diff;
            heartbeat_rate = 60000 * BUFFER_SIZE / (heartbeat_diff_sum * 20);
            heartbeat_buf_idx++;
            if (heartbeat_buf_idx > BUFFER_SIZE)
                heartbeat_buf_idx = 0;
        }
        heartbeat_none_cnt = 0;
        heartbeat_time_pre = heartbeat_time;
    }
    else
    {
        heartbeat_none_cnt++;
        if (heartbeat_none_cnt > 10000/ADC_TRIGGER_PERIOD)
        {
            heartbeat_rate = 0;
        }
    }

    breath_time = breath_rate_analysis(x, time_idx);
    if (breath_time > 0)
    {
        breath_cnt++;
        breath_diff = breath_time - breath_time_pre;
        if ((breath_diff < 60000/5/ADC_TRIGGER_PERIOD) && (breath_diff > 60000/60/ADC_TRIGGER_PERIOD))// 5 < rate < 60
        {
            breath_diff_sum -= breath_buf[breath_buf_idx];
            breath_buf[breath_buf_idx] = breath_diff;
            breath_diff_sum += breath_buf[breath_buf_idx];

            breath_diff_average = breath_diff_sum / BUFFER_SIZE;
            breath_rate = 60000 / (breath_diff_average * ADC_TRIGGER_PERIOD);
            //breath_rate = 60000 * BUFFER_SIZE / (breath_diff_sum * 20);
            breath_buf_idx++;
            if (breath_buf_idx >= BUFFER_SIZE)
                breath_buf_idx = 0;
        }
        //breath_rate = 3000 / (breath_diff);

        breath_none_cnt = 0;
        breath_time_pre = breath_time;
    }
    else
    {
        breath_none_cnt++;
        if (breath_none_cnt > 10000/ADC_TRIGGER_PERIOD) // 10 sec
        {
            breath_rate = 0;
        }
    }

    turning_tmp = turning_analysis(x, time_idx);
    if (turning_tmp > 0)
    {
        turning_cnt++;
        turning_flag = 1;
    }

}