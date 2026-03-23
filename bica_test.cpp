/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_test.c
 * Created: Oct 10, 2025
 * Last Edited: Oct 20, 2025
 * 
 * Tests for BICA.
*/
#include "bica.h"

int num_tests = 0;
int num_passed = 0;

/* ################ */
/* HELPER FUNCTIONS */
/* ################ */
void test_start(int test_no, char* test_name){
    num_tests++;
    printf("------------------------\nTEST %d: %s\n\n", test_no, test_name);
}

void test_end(int success){
    if(success) num_passed++;
    printf("\nTEST %s | %d/%d Running Total\n\n", success? "PASS":"FAIL", num_passed, num_tests);
}

void print_bica_arr(unsigned char * arr, int size){
    printf("BICAMSG{");
    for(int i = 0; i < size; i++)
        printf("%02x%s", arr[i], (i == size-1)?"": " ");
    printf("}\n");
}

/* ################# */
/* TESTING FUNCTIONS */
/* ################# */
// test_blank message generation (and lookup)
void test_1(){
    test_start(1, "M_TEST_DUMMY Create");
    bica_func function;
    function = bica_get_function(BICAM_TEST_DUMMY, BICAT_CREATE);

    if(function == nullptr){ 
        test_end(0);
    }else{
        unsigned char buf[BICA_BUFFER_LEN];
        function(buf, BICA_BUFFER_LEN, NULL);
        print_bica_arr(buf, BICA_BUFFER_LEN);
        for (int i = 1; i< BICA_BUFFER_LEN; i++){
            if(buf[i] != i){
                test_end(0);
                return;
            }
        }
        if(BICA_BUFFER_LEN>0){
            if(buf[0]!=0xFF){
                test_end(0);
                return;
            }
        }
        test_end(1);
    }
}

//Fail to find result
void test_2(){
    test_start(2, "M_TEST_BLANK FAIL TO FIND");
    bica_func function;
    function = bica_get_function(BICAM_TEST_NULLPTR, BICAT_PROCESS);

    test_end(function == nullptr);
}

//Buffer allocation
void test_3(){
    test_start(3, "bica_allocate_buffer()");
    unsigned char * buf = bica_allocate_buffer();
    print_bica_arr(buf, BICA_BUFFER_LEN);

    test_end(buf != nullptr);
    if(buf!=nullptr) free(buf);
}

//Adding a function and retrieving it
int dummy_var = 0;
int dummy_func(unsigned char* buffer, int buffer_len, void* data){
    dummy_var = 1;
    return 1;
}

void test_4(){
    test_start(4, "Modify BICA Table");
    bica_set_hook(BICAM_TEST_DUMMY, BICAT_PROCESS, &dummy_func);
    dummy_var = 0;

    bica_func function;
    function = bica_get_function(BICAM_TEST_DUMMY, BICAT_PROCESS);
    unsigned char buf[BICA_BUFFER_LEN];
    if(function == nullptr) printf("Bad Function Lookup\n");
    else function(buf, BICA_BUFFER_LEN, NULL);
    printf("Dummy Variable: %s\n",dummy_var?"Success":"Fail");

    test_end(dummy_var);
    dummy_var = 0;
    bica_set_hook(BICAM_TEST_DUMMY, BICAT_PROCESS, nullptr);
}

//Indexing/binary search test
void test_5(){
    test_start(5, "Check All Lookups");
    int pass = 1;
    for (int i = 0; i < BICA_NUM_MESSAGE_IDS; i++){
        int index_pass = _bica_get_index(_bica_m_lookup_table[i].message_id)==i;
        printf("0x%02x: %s\n", _bica_m_lookup_table[i].message_id, index_pass?"PASS":"FAIL");
        if(!index_pass)
            pass = 0;
        
    }
    test_end(pass);
}

//Overwriting on_nullptr
int dummy_var_2 = 0;
void dummy_func_2(unsigned char message_id, int type, int index_found){
    printf("Called new nullptr func, %02x\n", message_id);
    dummy_var_2 = 1;
}

void test_6(){
    test_start(6, "set bica_on_nullptr");
    bica_get_function(BICAM_TEST_DUMMY, BICAT_PROCESS);
    printf("Overwrite on_nullptr\n");
    bica_on_nullptr = dummy_func_2;
    bica_get_function(BICAM_TEST_DUMMY, BICAT_PROCESS);

    test_end(dummy_var_2);
    dummy_var_2 = 0;
    bica_on_nullptr = _bicad_on_nullptr;
}

void test_7(){
    test_start(7, "bicad handshakes");
    printf("BICA_VERSION: %d\n", BICA_VERSION);
    unsigned char buf[BICA_BUFFER_LEN];
    if(bica_get_function(BICAM_HANDSHAKE_REQ, BICAT_CREATE)(buf, BICA_BUFFER_LEN, nullptr) == 0){
        printf("Failed to generate handshake req\n");
        test_end(0);
        return;
    }
    print_bica_arr(buf, BICA_BUFFER_LEN);
    if(buf[0]!= BICAM_HANDSHAKE_REQ || buf[1] != BICA_VERSION){
        printf("Incorrect Buffer handshake req, fail\n");
        test_end(0);
        return;
    }

    if(bica_get_function(BICAM_HANDSHAKE_REP, BICAT_CREATE)(buf, BICA_BUFFER_LEN, nullptr) == 0){
        printf("Failed to generate handshake rep\n");
        test_end(0);
        return;
    }
    print_bica_arr(buf, BICA_BUFFER_LEN);
    if(buf[0]!= BICAM_HANDSHAKE_REP || buf[1] != BICA_VERSION){
        printf("Incorrect Buffer handshake rep, fail\n");
        test_end(0);
        return;
    }

    test_end(1);
}

/* ######################### */
/* CONTROL TESTING FUNCTIONS */
/* ######################### */

#include "bica_control/controller/include/common.h"
#include "bica_control/bica_control_bcu.h"
#include "bica_control/bica_control_main.h"

uint8_t transmission_buffer[BICA_BUFFER_LEN];
int transmit_flag = 0;

void test_8(){
    test_start(8, "BICA bcu control setup");
    int success_flag;
    success_flag = bica_init_bcu_control(nullptr,
                                         [](State x, State y, Pose a)->int{return 1;}, 
                                         [](Input i)->int{return 1;}
                                        );
    if(success_flag == EOK){
        printf("Allowed null bcu init _send_callback\n");
        test_end(0);
        return;
    }

    success_flag = bica_init_bcu_control([](unsigned char * buffer, int buf_len)->int{return 1;}, 
                                         nullptr, 
                                         [](Input i)->int{return 1;}
                                        );
    if(success_flag == EOK){
        printf("Fail: Allowed null bcu init _ctrl_callback_states\n");
        test_end(0);
        return;
    }

    success_flag = bica_init_bcu_control([](unsigned char * buffer, int buf_len)->int{return 1;}, 
                                         nullptr, 
                                         [](Input i)->int{return 1;}
                                        );
    if(success_flag == EOK){
        printf("Fail: Allowed null bcu init _ctrl_callback_inputs\n");
        test_end(0);
        return;
    }

    success_flag = bica_init_bcu_control([](unsigned char * buffer, int buf_len)->int{return 1;}, 
                                         [](State x, State y, Pose a)->int{return 1;}, 
                                         [](Input i)->int{return 1;});
    if(success_flag != EOK){
        printf("Fail: disallowed correct bica_init_bcu_control with errno %d\n", success_flag);
        test_end(0);
        return;
    }else{
        printf("Successful bica_init_bcu_control\n");
    }
    

    test_end(1);
}

void test_9(){
    test_start(9, "BICA main control setup");
    int success_flag;
    success_flag = bica_init_main_control(nullptr);
    if(success_flag == EOK){
        printf("Allowed null main init _send_callback\n");
        test_end(0);
        return;
    }

    success_flag = bica_init_main_control([](unsigned char * buffer, int buf_len)->int{return 1;});
    if(success_flag != EOK){
        printf("Fail: disallowed correct bica_init_main_control with errno %d\n", success_flag);
        test_end(0);
        return;
    }else{
        printf("Successful bica_init_main_control\n");
    }
    

    test_end(1);
}
int success_flag, receive_flag;
State current, desired;
Pose accel;
#include <thread>
#include <chrono>

void test_10(){
    test_start(10, "BICA control state passalong");

    //Init a success flag
    success_flag = 1;
    receive_flag = 0;

    // Create simulated "transmission" function
    auto transmit_func = [](uint8_t * buffer, int buffer_len)->int{
        transmit_flag = true;
        for(int i = 0; i < buffer_len && i < BICA_BUFFER_LEN; i++)
            transmission_buffer[i] = buffer[i];
        return 1;
    };

    // Create a "dummy" current state, desired state, and acceleration to pass along
    current = State(Pose(0.0, 0.1, 0.2, 0.3), Pose(0.4, 0.5, 0.6, 0.7));
    desired = State(Pose(1.0, 1.1, 1.2, 1.3), Pose(1.4, 1.5, 1.6, 1.7));
    accel = Pose(Pose(2.0, 2.1, 2.2, 0));

    // Create evaluation functions to 'slot' into the controller's place
    auto state_eval = [](State _c, State _d, Pose _a)->int{
        #define SUCCESS_MARGIN 0.001
        receive_flag = 1;
        success_flag &= abs(_c.pose.x - current.pose.x) < SUCCESS_MARGIN;
        success_flag &= abs(_c.pose.y - current.pose.y) < SUCCESS_MARGIN;
        success_flag &= abs(_c.pose.z - current.pose.z) < SUCCESS_MARGIN;
        success_flag &= abs(_c.pose.theta - current.pose.theta) < SUCCESS_MARGIN;
        success_flag &= abs(_c.velocity.x - current.velocity.x) < SUCCESS_MARGIN;
        success_flag &= abs(_c.velocity.y - current.velocity.y) < SUCCESS_MARGIN;
        success_flag &= abs(_c.velocity.z - current.velocity.z) < SUCCESS_MARGIN;
        success_flag &= abs(_c.velocity.theta - current.velocity.theta) < SUCCESS_MARGIN;
        
        success_flag &= abs(_d.pose.x - desired.pose.x) < SUCCESS_MARGIN;
        success_flag &= abs(_d.pose.y - desired.pose.y) < SUCCESS_MARGIN;
        success_flag &= abs(_d.pose.z - desired.pose.z) < SUCCESS_MARGIN;
        success_flag &= abs(_d.pose.theta - desired.pose.theta) < SUCCESS_MARGIN;
        success_flag &= abs(_d.velocity.x - desired.velocity.x) < SUCCESS_MARGIN;
        success_flag &= abs(_d.velocity.y - desired.velocity.y) < SUCCESS_MARGIN;
        success_flag &= abs(_d.velocity.z - desired.velocity.z) < SUCCESS_MARGIN;
        success_flag &= abs(_d.velocity.theta - desired.velocity.theta) < SUCCESS_MARGIN;

        success_flag &= abs(_a.x - accel.x) < SUCCESS_MARGIN;
        success_flag &= abs(_a.y - accel.y) < SUCCESS_MARGIN;
        success_flag &= abs(_a.z - accel.z) < SUCCESS_MARGIN;

        printf("Expect: p%f %f %f %f \n", current.pose.x, current.pose.y, current.pose.z, current.pose.theta);
        printf("      | v%f %f %f %f \n", current.velocity.x, current.velocity.y, current.velocity.z, current.velocity.theta);
        printf("      | p%f %f %f %f \n", desired.pose.x, desired.pose.y, desired.pose.z, desired.pose.theta);
        printf("      | v%f %f %f %f \n", desired.velocity.x, desired.velocity.y, desired.velocity.z, desired.velocity.theta);
        printf("      | a%f %f %f \n", accel.x, accel.y, accel.z);

        printf("Recvd.: p%f %f %f %f \n", _c.pose.x, _c.pose.y, _c.pose.z, _c.pose.theta);
        printf("      | v%f %f %f %f \n", _c.velocity.x, _c.velocity.y, _c.velocity.z, _c.velocity.theta);
        printf("      | p%f %f %f %f \n", _d.pose.x, _d.pose.y, _d.pose.z, _d.pose.theta);
        printf("      | v%f %f %f %f \n", _d.velocity.x, _d.velocity.y, _d.velocity.z, _d.velocity.theta);
        printf("      | a%f %f %f \n", _a.x, _a.y, _a.z);
        return 0;
    };
    auto input_eval = [](Input _i)->int{
        success_flag = 0;
        printf("ERROR Called wrong controller callback.\n");
        return 0;
    };


    bica_init_main_control(transmit_func);
    bica_init_bcu_control(transmit_func, state_eval, input_eval);

    _bica_m_function_ptr func1 =  bica_get_function(BICAM_SEND_CONTROL_UPD, BICAT_CREATE);
    if(func1 != nullptr){
        _control_buffer *buf;
        bica_create_control_buffer(&buf, current, desired, accel);
        printf("Processing 0x45C ");
        func1(transmission_buffer, BICA_BUFFER_LEN, buf);
        print_bica_arr(transmission_buffer, BICA_BUFFER_LEN);
        
        transmit_flag = true;
        int num_loops = 0;
        #define MAX_LOOPS 100
        while(transmit_flag && num_loops++ < MAX_LOOPS){
            _bica_m_function_ptr func2 = bica_get_function(transmission_buffer[0], BICAT_PROCESS);
            if(func2 == nullptr){
                success_flag = 0;
                printf("Failed to lookup processing function 0x%x\n", transmission_buffer[0]);
                break;
            }

            printf("Processing 0x%xP ", transmission_buffer[0]);
            transmit_flag = false;
            int status = func2(transmission_buffer, BICA_BUFFER_LEN, nullptr);
            print_bica_arr(transmission_buffer, BICA_BUFFER_LEN);

            if(status != EOK){
                printf("Function 0x%xP failed with code %d\n", transmission_buffer[0], status);
                success_flag = 0;
                break;
            }
        }
        if(num_loops >= MAX_LOOPS){
            success_flag = 0;
            printf("Did not succeed withing %d loops.", MAX_LOOPS);
        }

    }else{
        printf("Invalid Function Call\n");
        success_flag = 0;   
    }
    
    test_end(success_flag & receive_flag);
}

Input input;

void test_11(){
    test_start(11, "BICA control input passalong");

    //Init a success flag
    success_flag = 1;
    receive_flag = 0;

    // Create simulated "transmission" function
    auto transmit_func = [](uint8_t * buffer, int buffer_len)->int{
        transmit_flag = true;
        for(int i = 0; i < buffer_len && i < BICA_BUFFER_LEN; i++)
            transmission_buffer[i] = buffer[i];
        return 1;
    };

    // Create a "dummy" current state, desired state, and acceleration to pass along
    input = Input(10.0, 50.0, 100.0);

    // Create evaluation functions to 'slot' into the controller's place
    auto state_eval = [](State _c, State _d, Pose _a)->int{
        success_flag = 0;
        printf("ERROR Called wrong controller callback.\n");
        return 0;
    };

    auto input_eval = [](Input _i)->int{
        #define SUCCESS_MARGIN 0.001
        receive_flag = 1;
        success_flag &= abs(_i.left - input.left) < SUCCESS_MARGIN;
        success_flag &= abs(_i.right - input.right) < SUCCESS_MARGIN;
        success_flag &= abs(_i.ballast - input.ballast) < SUCCESS_MARGIN;
        printf("Expect: i%f %f %f \n", input.left, input.right, input.ballast);
        printf("Recvd.: i%f %f %f \n", _i.left, _i.right, _i.ballast);
    };

    bica_init_main_control(transmit_func);
    bica_init_bcu_control(transmit_func, state_eval, input_eval);

    _bica_m_function_ptr func1 =  bica_get_function(BICAM_SEND_CONTROL_UPD, BICAT_CREATE);
    if(func1 != nullptr){
        _control_buffer *buf;
        bica_create_control_buffer(&buf, input);
        printf("Processing 0x45C ");
        func1(transmission_buffer, BICA_BUFFER_LEN, buf);
        print_bica_arr(transmission_buffer, BICA_BUFFER_LEN);
        
        transmit_flag = true;
        int num_loops = 0;
        #define MAX_LOOPS 100
        while(transmit_flag && num_loops++ < MAX_LOOPS){
            _bica_m_function_ptr func2 = bica_get_function(transmission_buffer[0], BICAT_PROCESS);
            if(func2 == nullptr){
                success_flag = 0;
                printf("Failed to lookup processing function 0x%x\n", transmission_buffer[0]);
                break;
            }

            printf("Processing 0x%xP ", transmission_buffer[0]);
            transmit_flag = false;
            int status = func2(transmission_buffer, BICA_BUFFER_LEN, nullptr);
            print_bica_arr(transmission_buffer, BICA_BUFFER_LEN);

            if(status != EOK){
                printf("Function 0x%xP failed with code %d\n", transmission_buffer[0], status);
                success_flag = 0;
                break;
            }
        }
        if(num_loops >= MAX_LOOPS){
            success_flag = 0;
            printf("Did not succeed withing %d loops.", MAX_LOOPS);
        }

    }else{
        printf("Invalid Function Call\n");
        success_flag = 0;   
    }
    
    test_end(success_flag & receive_flag);
}


int main(int argc, char* argv[]){
    // Main BICA tests
    test_1();
    test_2();
    test_3();
    test_4();
    test_5();
    test_6();
    test_7();

    // Control BICA tests
    test_8();
    test_9();
    test_10();
    test_11();
}