
#include "network.h"

void *context = zmq_ctx_new();
#ifndef ZMQ_BUILD_DRAFT_API
void *publisher = zmq_socket(context, ZMQ_PUB);
void *receiver = zmq_socket(context, ZMQ_SUB);
#else
void *publisher = zmq_socket(context, ZMQ_RADIO);
void *receiver = zmq_socket(context, ZMQ_DISH);
#endif

struct SendData Txbuffer;
struct RecvData Rxbuffer;

struct UDPFragment RxBuffer_Frag[RECV_FRAGMENT_NUM];
struct UDPFragment TxBuffer_frag[SEND_FRAGMENT_NUM];

zmq_msg_t Tx_msg;
zmq_msg_t Rx_msg;
const char *Tx_group = "uav2master";//, "uav2master1", "uav2master2", "uav2master3"};
const char *Rx_group = "master2uav";

#define SETTLE_TIME_US  300000          /// set settle time 300ms, accroding to official demo

#ifndef ZMQ_BUILD_DRAFT_API
int network_depth_log_initial()
{
    int rc = zmq_connect(receiver, "tcp://166.111.74.88:50608");
    assert(rc == 0);
    rc = zmq_setsockopt(receiver, ZMQ_SUBSCRIBE, "50608", 5);
    assert(rc == 0);
    rc = zmq_bind(publisher, "tcp://*:50603");
    assert(rc == 0);
    return 0;
}
#else
int network_depth_log_initial(string master_addr)
{
    int rc = zmq_bind(receiver, "udp://*:50608");
    assert (rc == 0);
    string pub_addr = "udp://" + master_addr + ":50603";
    rc = zmq_connect(publisher, pub_addr.c_str());
    assert (rc == 0);
    usleep(SETTLE_TIME_US);
//    int64_t udp_msg_size;
//    size_t size = sizeof(int64_t);
//    rc = zmq_getsockopt(publisher, ZMQ_MAXMSGSIZE, &udp_msg_size, &size);
//    printf("UDP message size = %d\n", udp_msg_size);
    rc = zmq_join(receiver, Rx_group);
    assert (rc == 0);
    for(int i=0; i<RECV_FRAGMENT_NUM; i++)
        RxBuffer_Frag[i].frame_idx = rand()%100;
}

#endif

void network_depth_log_send()
{
    Txbuffer.flag[0] = '5';
    Txbuffer.flag[1] = '0';
    Txbuffer.flag[2] = '6';
    Txbuffer.flag[3] = '0';
    Txbuffer.flag[4] = '3';
    Txbuffer.flag[5] = '\0';
#ifndef ZMQ_BUILD_DRAFT_API
    zmq_send(publisher, &Txbuffer, sizeof(struct SendData), 0);
#else
    int rc = network_udp_msg_send();
    assert(rc != -1);
#endif
}


void network_make_buffer(int frame_num, double pose[3], double q[4], int drone_action_flag)
{
    Txbuffer.frame_num = frame_num;
  	struct timeval time_now;
    Txbuffer.sec = time_now.tv_sec;
    Txbuffer.usec = time_now.tv_usec;

    Txbuffer.drone_action_flag = drone_action_flag;

    Txbuffer.q_send[0] = q[0];
    Txbuffer.q_send[1] = q[1];
    Txbuffer.q_send[2] = q[2];
    Txbuffer.q_send[3] = q[3];

    Txbuffer.pose[0] = (float)pose[0];
    Txbuffer.pose[1] = (float)pose[1];
    Txbuffer.pose[2] = (float)pose[2];

    Txbuffer.ultra_height = (float)pose[2];
}

#ifdef ZMQ_BUILD_DRAFT_API
int network_udp_msg_send(void)
{
    // Prepare sending data
    for(int i=0; i<SEND_FRAGMENT_NUM; i++){
        TxBuffer_frag[i].frame_idx = Txbuffer.frame_num;
        TxBuffer_frag[i].fragment_idx = i;
        if(i != SEND_FRAGMENT_NUM-1)
            TxBuffer_frag[i].data_len = NORMAL_SEND_FRAG_LEN;
        else
            TxBuffer_frag[i].data_len = LAST_SEND_FRAG_LEN;
        memcpy(TxBuffer_frag[i].data, (char *)(&Txbuffer) + NORMAL_SEND_FRAG_LEN*i, TxBuffer_frag[i].data_len);
    }
    int rc;
    for(int i=0; i<SEND_FRAGMENT_NUM; i++) {
        unsigned int fragment_len = TxBuffer_frag[i].data_len + (unsigned int)sizeof(unsigned int)*3;
        rc = zmq_msg_init_size(&Tx_msg, fragment_len);
        assert(fragment_len < 1<<13);
        if (rc != 0) {
            printf("message initialization failed\n");
            return rc;
        }
        memcpy(zmq_msg_data(&Tx_msg), (void *)(&TxBuffer_frag[i]), fragment_len);
        rc = zmq_msg_set_group(&Tx_msg, Tx_group);
        if (rc != 0) {
            printf("message set group failed\n");
            zmq_msg_close(&Tx_msg);
            return rc;
        }
        rc = zmq_msg_send(&Tx_msg, publisher, 0);
//        printf("zmq_msg_send = %d\n", rc);
        zmq_msg_close(&Tx_msg);
    }
    return rc;
}

int network_udp_msg_receive(void)
{
    int rc = network_udp_msg_receive_fragment();
    assert(rc != -1);
    int frame_idx = RxBuffer_Frag[0].frame_idx;
    for(int i=0; i<RECV_FRAGMENT_NUM; i++){
        if(frame_idx != RxBuffer_Frag[i].frame_idx)
            return 0;
    }
    // Whole pack received
    for(int i=0; i<RECV_FRAGMENT_NUM; i++){
        assert(RxBuffer_Frag[i].fragment_idx == i);
        memcpy((char *)(&Rxbuffer)+ NORMAL_RECV_FRAG_LEN*i, RxBuffer_Frag[i].data, RxBuffer_Frag[i].data_len);
    }
//    printf("Frame index = %d\n", frame_idx);
    return 1;
}

int msg_send (zmq_msg_t *msg_, void *s_, const char* group_, const char* body_)
{
    int rc = zmq_msg_init_size (msg_, strlen (body_));
    printf("strlen (body_) = %d\n", (int)strlen (body_));
    if (rc != 0)
        return rc;

    memcpy (zmq_msg_data (msg_), body_, strlen (body_));

    rc = zmq_msg_set_group (msg_, group_);
    if (rc != 0) {
        zmq_msg_close (msg_);
        return rc;
    }

    rc = zmq_msg_send (msg_, s_, 0);
//    printf("zmq_msg_send = %d\n", rc);
    zmq_msg_close (msg_);

    return rc;
}

int network_udp_msg_receive_fragment(void)
{
    static UDPFragment temp_frag;
    int rc = zmq_msg_init(&Rx_msg);
    if (rc != 0) {
        printf("network.cpp [UDP debug]: message initialization failed\n");
        return -1;              /// message initialization failed
    }

    int recv_rc = 0;
    recv_rc = zmq_msg_recv(&Rx_msg, receiver, 0);
    if (recv_rc == -1) {
        zmq_msg_close(&Rx_msg);
        printf("network.cpp [UDP debug]: message receive failed\n");
        return -2;              /// message receive failed
    }

    if (strcmp(zmq_msg_group(&Rx_msg), Rx_group) != 0)
    {
        zmq_msg_close(&Rx_msg);
        printf("network.cpp [UDP debug]: message group not match\n");
        return -3;              /// message group not match
    }

    memcpy((void*)(&temp_frag), zmq_msg_data(&Rx_msg), zmq_msg_size(&Rx_msg));

    unsigned int msg_size = (unsigned int)zmq_msg_size(&Rx_msg);
    unsigned int temp_frag_size = temp_frag.data_len + (unsigned int)sizeof(unsigned int)*3;
    if (msg_size != temp_frag_size)
    {
        zmq_msg_close(&Rx_msg);
        printf("network.cpp [UDP debug]: message size not match;\n");
        printf("zmq_msg_size(&Rx_msg) = %d, sizeof(struct UDPFragment) = %d\n",msg_size , temp_frag_size);
        return -4;              /// message size not match
    }

    zmq_msg_close(&Rx_msg);

    if (temp_frag.fragment_idx >= 0 && temp_frag.fragment_idx < RECV_FRAGMENT_NUM){
        memcpy(&RxBuffer_Frag[temp_frag.fragment_idx], &temp_frag, temp_frag_size);
    }

    return recv_rc;
}

int msg_recv_cmp()
{
    zmq_msg_t *msg_ = &Rx_msg;
    void *s_ = receiver;
    const char* group_ = Tx_group;
    const char* body_ = "Hello from master!";
    int rc = zmq_msg_init (msg_);
    if (rc != 0)
        return -1;
    int recv_rc = zmq_msg_recv (msg_, s_, 0);
    if (recv_rc == -1) {
        zmq_msg_close(msg_);
        return -1;
    }
    if (strcmp (zmq_msg_group (msg_), group_) != 0)
    {
        zmq_msg_close (msg_);
        return -1;
    }
    char * body = (char*) malloc (sizeof(char) * (zmq_msg_size (msg_) + 1));
    memcpy (body, zmq_msg_data (msg_), zmq_msg_size (msg_));
    body [zmq_msg_size (msg_)] = '\0';

    zmq_msg_close (msg_);
    free (body);

    return recv_rc;
}

#endif