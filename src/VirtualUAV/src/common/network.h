#ifndef __COMMON_NETWORK_H__
#define __COMMON_NETWORK_H__

#include <iostream>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <cstring>
#include <stdlib.h>
#define ZMQ_BUILD_DRAFT_API
#include <zmq.h>

using namespace std;

extern void *context;
extern void *publisher;
struct SendData{
    char flag [6];
    unsigned int sec; // time stamp
    unsigned int usec;  // time stamp 
    int frame_num; 
    char depth[320 * 240 * 2];

    int drone_action_flag;

    float q_send[4];
    float pose[3];

    float ultra_height;
};

struct RecvData{
    char flag[6];
    int frame_num;
    int node_num;
    float nbv[8];
    float box[24];
    float Tv2d[12];
    float node_pc[4096][3];
};

#ifdef ZMQ_BUILD_DRAFT_API

#define SEND_FRAGMENT_NUM       20
#define NORMAL_SEND_FRAG_LEN    ((int)sizeof(struct SendData)/SEND_FRAGMENT_NUM)
#define LAST_SEND_FRAG_LEN      ((int)sizeof(struct SendData) - NORMAL_SEND_FRAG_LEN * (SEND_FRAGMENT_NUM - 1))

#define RECV_FRAGMENT_NUM       8
#define NORMAL_RECV_FRAG_LEN    ((int)sizeof(struct RecvData)/RECV_FRAGMENT_NUM)
#define LAST_RECV_FRAG_LEN      ((int)sizeof(struct RecvData) - NORMAL_SEND_FRAG_LEN * (RECV_FRAGMENT_NUM - 1))


struct UDPFragment{
    unsigned int frame_idx;
    unsigned int fragment_idx;
    unsigned int data_len;
    char data[8192]; // Maximum length of UDP in Linux
};
#endif


extern struct RecvData Rxbuffer;
extern struct SendData Txbuffer;
extern void *receiver;


int network_depth_log_initial(string master_addr); // Open Port
void network_depth_log_send(); // Send data
void network_make_buffer(int frame_num, double pose[3], double q[4], int drone_action_flag);
int network_udp_msg_send(void);
int network_udp_msg_receive(void);
int network_udp_msg_receive_fragment(void);
int msg_send (zmq_msg_t *msg_, void *s_, const char* group_, const char* body_);
int msg_recv_cmp ();
#endif // __COMMON_NETWORK_H__
