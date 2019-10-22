/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_dig_out = NULL;
static ec_slave_config_state_t sc_dig_out_state = {};

static ec_slave_config_t *sc_dig_in = NULL;
static ec_slave_config_state_t sc_dig_in_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos  0, 0 // Info aus "ethercat slaves"
#define DigOutSlavePos 0, 1
#define DigInSlavePos  0, 2

#define Beckhoff_EK1100 0x00000002, 0x044c2c52  // Info aus "ethercat slaves -v"
#define Beckhoff_EL2008 0x00000002, 0x07d83052  // ethercat cstruct -p1 -v -> "Vendor ID" und "Product Code"
#define Beckhoff_EL1008 0x00000002, 0x03f03052  // ethercat cstruct -p2 -v -> "Vendor ID" und "Product Code"


// offsets for PDO entries
static unsigned int off_dig_out;
static unsigned int off_dig_in;

const static ec_pdo_entry_reg_t domain1_regs[] = {              // ethercat cstruct -pX -v
    {DigOutSlavePos, Beckhoff_EL2008, 0x7000, 1, &off_dig_out}, // 0x7000 -> erster offset von ec_pdo_entry_info_t !!!
	{DigInSlavePos,  Beckhoff_EL1008, 0x6000, 1, &off_dig_in},  // 0x6000 -> erster offset von ec_pdo_entry_info_t !!!
    {}
};

static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

// Digital out ------------------------ Infos aus ethercat cstruct -p1 -v

static ec_pdo_entry_info_t el2008_channels[] = {
    {0x7000, 0x01, 1}, // Value 1
    {0x7010, 0x01, 1}, // Value 2
    {0x7020, 0x01, 1}, // Value 3
    {0x7030, 0x01, 1}, // Value 4
    {0x7040, 0x01, 1}, // Value 5
    {0x7050, 0x01, 1}, // Value 6
    {0x7060, 0x01, 1}, // Value 7
    {0x7070, 0x01, 1}  // Value 8
};

static ec_pdo_info_t el2008_pdos[] = {
    {0x1600, 1, &el2008_channels[0]},
    {0x1601, 1, &el2008_channels[1]},
    {0x1602, 1, &el2008_channels[2]},
    {0x1603, 1, &el2008_channels[3]},
    {0x1604, 1, &el2008_channels[4]},
    {0x1605, 1, &el2008_channels[5]},
    {0x1606, 1, &el2008_channels[6]},
    {0x1607, 1, &el2008_channels[7]}
};

// Digital out ------------------------ Infos aus ethercat cstruct -p2 -v

static ec_pdo_entry_info_t el1008_channels[] = {
    {0x6000, 0x01, 1}, // Value 1
    {0x6010, 0x01, 1}, // Value 2
    {0x6020, 0x01, 1}, // Value 3
    {0x6030, 0x01, 1}, // Value 4
    {0x6040, 0x01, 1}, // Value 5
    {0x6050, 0x01, 1}, // Value 6
    {0x6060, 0x01, 1}, // Value 7
    {0x6070, 0x01, 1}  // Value 8
};

static ec_pdo_info_t el1008_pdos[] = {
    {0x1a00, 1, &el1008_channels[0]},
    {0x1a01, 1, &el1008_channels[1]},
    {0x1a02, 1, &el1008_channels[2]},
    {0x1a03, 1, &el1008_channels[3]},
    {0x1a04, 1, &el1008_channels[4]},
    {0x1a05, 1, &el1008_channels[5]},
    {0x1a06, 1, &el1008_channels[6]},
    {0x1a07, 1, &el1008_channels[7]}
};

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_dig_out, &s);

    if (s.al_state != sc_dig_out_state.al_state) {
        printf("DigOut: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_dig_out_state.online) {
        printf("DigOut: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_dig_out_state.operational) {
        printf("DigOut: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_dig_out_state = s;
}

/*****************************************************************************/

void cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state
    check_domain1_state();

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;

        // calculate new process data
        blink = !blink;

        // check for master state (optional)
        check_master_state();

        // check for slave configuration state(s) (optional)
        check_slave_config_states();
    }

    // write process data
    EC_WRITE_U8(domain1_pd + off_dig_out, blink ? 0x06 : 0x09);

    // read process data
    printf("DigIn: Value 0x%02x\n", EC_READ_U8(domain1_pd + off_dig_in));

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }


    if (!(sc_dig_out = ecrt_master_slave_config( master, DigOutSlavePos, Beckhoff_EL2008)) ) {
        fprintf(stderr, "Failed to get slave configuration form EL2008.\n");
        return -1;
    }

//    if (ecrt_slave_config_pdos(sc, EC_END, el2008_syncs)) {
//        fprintf(stderr, "Failed to configure PDOs.\n");
//        return -1;
//    }

    if (!(sc_dig_in = ecrt_master_slave_config( master, DigInSlavePos, Beckhoff_EL1008)) ) {
        fprintf(stderr, "Failed to get slave configuration from EL1008.\n");
        return -1;
    }

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc) {
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}

/****************************************************************************/

