/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


// function declarations
void rotate(float angle, int direction, int speed);
void pivot(int angle, int direction, int pivot_wheel, int speed);
void stabilized_move(float distance, int direction, int speed, float wall_distance);
void align(int count);
int look_for_block();
float Front_Left_Ultrasonic_Sensor(int sample_time, int use_median);
float Front_Right_Ultrasonic_Sensor(int sample_time, int use_median);
float Left_Ultrasonic_Sensor(int sample_time, int use_median);
float Right_Ultrasonic_Sensor(int sample_time, int use_median);
float Side_Ultrasonic_Sensor(int sample_time);
float calculate_median_distance(float * distance_array, int counts);
void close_grippers();
void stop_motors();
void move_forward(int speed);
void move_backward(int speed);
void reset_motor_counts();
void position_after_block_search(int location);
int look_for_pucks(int position);
int pick_up_puck(int position, int puck_position, int repeat);
int pick_puck(int steps, int depth);
void get_color_order();
void return_with_puck(int arena_position, int color);
void stack_pucks(int position);
void stack_puck(int arena_position);

// Debugging variables
char buf[1000];

// Motor encoder variables
int left_motor_count = 0;
int right_motor_count = 0;
int max_lmc = 100000;
int max_rmc = 100000;
int stop_all_motors = 1;
int motors_running = 0;


// ultrasonic variables
float us_error_margin = 0.01; // acceptable level of difference in ultrasonic values (in meters)
float us_sum_front_left = 0.0;
int us_counts_front_left = 0.0;
float us_values_front_left[2500];
float us_sum_front_right = 0.0;
int us_counts_front_right = 0.0;
float us_values_front_right[2500];
float us_sum_left = 0.0;
int us_counts_left = 0.0;
float us_values_left[2500];
float us_sum_right = 0.0;
int us_counts_right = 0.0;
float us_values_right[2500];
uint16 count_front_right = 0;
uint16 count_front_left = 0;
uint16 count_left = 0;
uint16 count_right = 0;

// position variables
int side = 1; // side closest to wall, 1 if left side, 0 if right side

// color variables
int current_puck = 0;
int puck_colors[3] = {0, 0, 0};
int puck_order[3] = {-1, -1, -1};

// puck picking variables
int distance_from_wall = 0;  // motor count of how far to travel when approaching horizontally
int pucks_picked = 0;
int pucks_stacked = 0;
int stash_positions[3] = {0, 0, 0};

const char* color_str[] = { "GREEN", "RED", "BLUE"};
int minimum_count_threshold = 9;
int minimum_color_threshold_2 = 25000;
int maximum_color_threshold_2 = 105000;
int minimum_color_threshold_1 = 25000;
int maximum_color_threshold_1 = 105000;

int color_test1(){
    
    MUX_Select(0);
    RED1_Write(0);
    GREEN1_Write(0);
    BLUE1_Write(0);
    int color_count[3] = {0,0,0};
    for(int k = 0;k<20;k++){
        RED1_Write(0);
        GREEN1_Write(0);
        BLUE1_Write(0);
        int i_result[3];
        int result[3];
        int color = -1;
        int max = -1000000000;
        for(int i=0;i<3;i++){
            RED1_Write(0);
            GREEN1_Write(0);
            BLUE1_Write(0);
            CyDelay(25);
            ADC_StartConvert();
            ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
            i_result[i] = ADC_GetResult32();
            if(i==0){
                RED1_Write(0);
                GREEN1_Write(1);
                BLUE1_Write(0);
            }
            if(i==1){
                RED1_Write(1);
                GREEN1_Write(0);
                BLUE1_Write(0);
            }
            if(i==2){
                RED1_Write(0);
                GREEN1_Write(0);
                BLUE1_Write(1);
            }
            
            ADC_StartConvert();
            ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
            result[i] = ADC_GetResult32();
            
            int reading = -result[i]+i_result[i];
            if (i==0){
                reading *= 1.1;
            }
            sprintf(buf,"1 %d:%d-%d=%d\n", i,result[i], i_result[i], reading);
            UART_PutString(buf);
            if (reading > max && reading > minimum_color_threshold_1 && reading < maximum_color_threshold_1 ){
                max = reading;
                color = i;
            }
            CyDelay(25);
            RED1_Write(0);
            GREEN1_Write(0);
            BLUE1_Write(0);
        }
        if(color != -1){
            color_count[color] += 1;
            sprintf(buf,"1 %d\n",color);
            UART_PutString(buf);
        }    
        
        RED1_Write(0);
        GREEN1_Write(0);
        BLUE1_Write(0);
    }
    int max1=-1,max2=-1;
    int loc1,loc2;
    for(int i = 0;i<3;i++){
        sprintf(buf,"1 %d\n",color_count[i]);
        UART_PutString(buf);
        if(color_count[i] > max1){
            max2 = max1;
            loc2 = loc1;
            max1 = color_count[i];
            loc1 = i;
        }else if(color_count[i] > max2){
            max2 = color_count[i];
            loc2 = i;
        }    
        
    }
    int dispcolor = -1;
    if(max1>=minimum_count_threshold && max1>max2){
        sprintf(buf,"1 %s\n",color_str[loc1]);
        UART_PutString(buf);
        dispcolor = loc1;
    }else{
        sprintf(buf,"1 NONE\n");
        UART_PutString(buf);
    }
    switch(dispcolor){
        case 0:
            RED1_Write(0);
            GREEN1_Write(1);
            BLUE1_Write(0);
            CyDelay(1000); 
            return 0;
        case 1:
            RED1_Write(1);
            GREEN1_Write(0);
            BLUE1_Write(0);  
            CyDelay(1000);
            return 1;
        case 2:
            RED1_Write(0);
            GREEN1_Write(0);
            BLUE1_Write(1);
            CyDelay(1000);
            return 2;
        default:
            RED1_Write(0);
            GREEN1_Write(0);
            BLUE1_Write(0);
            CyDelay(1000);
            return 3;
    }
    CyDelay(1000);
}


int color_test2(){
    MUX_Select(1);
    RED2_Write(0);
    GREEN2_Write(0);
    BLUE2_Write(0);
    int color_count[3] = {0,0,0};
    for(int k = 0;k<20;k++){
        RED2_Write(0);
        GREEN2_Write(0);
        BLUE2_Write(0);
        int i_result[3];
        int result[3];
        int color = -1;
        int max = -1000000000;
        for(int i=0;i<3;i++){
            RED2_Write(0);
            GREEN2_Write(0);
            BLUE2_Write(0);
            CyDelay(25);
            ADC_StartConvert();
            ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
            i_result[i] = ADC_GetResult32();
            if(i==0){
                RED2_Write(0);
                GREEN2_Write(1);
                BLUE2_Write(0);
            }
            if(i==1){
                RED2_Write(1);
                GREEN2_Write(0);
                BLUE2_Write(0);
            }
            if(i==2){
                RED2_Write(0);
                GREEN2_Write(0);
                BLUE2_Write(1);
            }
            
            ADC_StartConvert();
            ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
            result[i] = ADC_GetResult32();
            
            int reading = -result[i]+i_result[i];
            if (i==0){
                reading *= 0.7;
            }else if (i==1){
                reading *= 1.2;
            }else if (i==2){
                reading *= 1;
            }
            sprintf(buf,"2 %d:%d-%d=%d\n", i,result[i], i_result[i], reading);
            UART_PutString(buf);
            if (reading > max && reading > minimum_color_threshold_2 && reading < maximum_color_threshold_2 ){
                max = reading;
                color = i;
            }
            CyDelay(25);
            RED2_Write(0);
            GREEN2_Write(0);
            BLUE2_Write(0);
        }
        if(color != -1){
            color_count[color] += 1;
            sprintf(buf,"2 %d\n",color);
            UART_PutString(buf);
        }    
        
        RED2_Write(0);
        GREEN2_Write(0);
        BLUE2_Write(0);
    }
    int max1=-1,max2=-1;
    int loc1,loc2;
    for(int i = 0;i<3;i++){
        sprintf(buf,"2 %d\n",color_count[i]);
        UART_PutString(buf);
        if(color_count[i] > max1){
            max2 = max1;
            loc2 = loc1;
            max1 = color_count[i];
            loc1 = i;
        }else if(color_count[i] > max2){
            max2 = color_count[i];
            loc2 = i;
        }    
        
    }
    int dispcolor = -1;
    if(max1>=minimum_count_threshold && max1>max2){
        sprintf(buf,"2 %s\n",color_str[loc1]);
        UART_PutString(buf);
        dispcolor = loc1;
    }else{
        sprintf(buf,"2 NONE\n");
        UART_PutString(buf);
    }
    switch(dispcolor){
        case 0:
            RED2_Write(0);
            GREEN2_Write(1);
            BLUE2_Write(0);
            CyDelay(1000); 
            return 0;
        case 1:
            RED2_Write(1);
            GREEN2_Write(0);
            BLUE2_Write(0);  
            CyDelay(1000);
            return 1;
        case 2:
            RED2_Write(0);
            GREEN2_Write(0);
            BLUE2_Write(1);
            CyDelay(1000);
            return 2;
        default:
            RED2_Write(0);
            GREEN2_Write(0);
            BLUE2_Write(0);
            CyDelay(1000);
            return 3;
    }
    CyDelay(1000);
}

void servo_motor_A(int degree)
{
    int val;
    // convert degrees to compare value
    //  10 to 200 === 0 to 180
    // then value is 
    val = degree;
    PWM_Servo_A_WriteCompare(val);
}

// Used to control what degree servo will work
void servo_motor_B(int degree)
{
    int val;
    // convert degrees to compare value
    //  10 to 200 === 0 to 180
    // then value is 
    val = degree;
    PWM_Servo_B_WriteCompare(val);
}


void servo_motor_C(int degree)
{
    int val;
    // convert degrees to compare value
    //  10 to 200 === 0 to 180
    // then value is 
    val = degree;
    PWM_Servo_C_WriteCompare(val);
}

void servo_motor_D(int degree)
{
    int val;
    // convert degrees to compare value
    //  10 to 200 === 0 to 180
    // then value is 
    val = degree;
    PWM_Servo_D_WriteCompare(val);
}


void close_grippers() {
    
    servo_motor_C(5);
    servo_motor_D(20);
    CyDelay(1000);
    
}

void open_grippers() {
    
    servo_motor_C(8);
    servo_motor_D(17);
    CyDelay(1000);
}

void open_grippers_wide() {
    
    servo_motor_C(10);
    servo_motor_D(15);
    CyDelay(100);
}

void raise() {
    for(int x = 0; x < 16; x++){ 
        servo_motor_B(20-x);
        servo_motor_A(5+x);
        CyDelay(100);
    }
    CyDelay(100);
}

void lower() {
    for(int x = 15; x >= 1; x--){ 
        servo_motor_B(20-x);
        servo_motor_A(5+x);
        CyDelay(100);
    }
    CyDelay(100);
}

void lower_lv(int h) {
    servo_motor_B(20-h*5);
    servo_motor_A(5+h*5);
    CyDelay(100);
}

void raise_lv() {
    servo_motor_B(5);
    servo_motor_A(20);
    CyDelay(100);
}

CY_ISR(Timer_ISR_Handler_Front_Left)
{
     //  Start the timer
    Timer_Front_Left_ReadStatusRegister();

    // Start the counter
    count_front_left = Timer_Front_Left_ReadCounter();

    float distance_measure = ((float)(65535 - count_front_left)) / 58.0 / 100.0;   // distance measured in m
    
    if (distance_measure <= 0) return;
    us_sum_front_left += distance_measure;
    us_values_front_left[us_counts_front_left] = distance_measure;
    us_counts_front_left += 1;
}


CY_ISR(Timer_ISR_Handler_Front_Right)
{
     //  Start the timer
    Timer_Front_Right_ReadStatusRegister();

    // Start the counter
    count_front_right = Timer_Front_Right_ReadCounter();

    float distance_measure = ((float)(65535 - count_front_right)) / 58.0 / 100.0;   // distance measured in m

    if (distance_measure <= 0) return;
    us_sum_front_right += distance_measure;
    us_values_front_right[us_counts_front_right] = distance_measure;
    us_counts_front_right += 1;
}


CY_ISR(Timer_ISR_Handler_Left)
{
     //  Start the timer
    Timer_Left_ReadStatusRegister();

    // Start the counter
    count_left = Timer_Left_ReadCounter();

    float distance_measure = ((float)(65535 - count_left)) / 58.0 / 100.0;   // distance measured in m

    if (distance_measure <= 0) return;
    us_sum_left += distance_measure;
    us_values_left[us_counts_left] = distance_measure;
    us_counts_left += 1;

}


CY_ISR(Timer_ISR_Handler_Right)
{
     //  Start the timer
    Timer_Right_ReadStatusRegister();
    // Start the counter
    count_right = Timer_Right_ReadCounter();

    float distance_measure = ((float)(65535 - count_right)) / 58.0 / 100.0;   // distance measured in m

    if (distance_measure <= 0) return;
    us_sum_right += distance_measure;
    us_values_right[us_counts_right] = distance_measure;
    us_counts_right += 1;
}
   



CY_ISR(left_motor_encoder)
{
    /*  Place your Interrupt code here. */
    left_motor_count += 1;
    if (left_motor_count >= max_lmc) 
    {
        PWM_LeftMotor_1_WriteCompare(0);
        PWM_LeftMotor_2_WriteCompare(0);
        if (stop_all_motors) 
        {
            PWM_RightMotor_1_WriteCompare(0);
            PWM_RightMotor_2_WriteCompare(0);    
        }
        motors_running = 0;
    }
    LeftMotor_Encoder_ISR_ClearPending();
    /* `#END` */
}

CY_ISR(right_motor_encoder)
{
    /*  Place your Interrupt code here. */
    right_motor_count += 1;
    if (right_motor_count >= max_rmc) 
    {
        PWM_RightMotor_1_WriteCompare(0);
        PWM_RightMotor_2_WriteCompare(0); 
        if (stop_all_motors) 
        {
            PWM_LeftMotor_1_WriteCompare(0);
            PWM_LeftMotor_2_WriteCompare(0);   
        }
        motors_running = 0;
    }
    RightMotor_Encoder_ISR_ClearPending();
    /* `#END` */
}



void InitializeHardware()
{
    Clock_1_Start();
    PWM_Servo_C_Start();
    PWM_Servo_A_Start();
    //PWM_Servo_Z_Start();
    PWM_Servo_B_Start();
    
    PWM_Servo_D_Start();
	CyGlobalIntEnable;		//	Comment this line to Disable global interrupts. */
    UART_Start();
    TIA_Start();
    ADC_Start();
    IDAC_Start();
    MUX_Start();
    
    //  Start the timer for Ultrasonic Sensor
    Timer_Front_Left_Start();
    Timer_Front_Right_Start();
    Timer_Left_Start();
    Timer_Right_Start();
    
    // Start all ultrasonic sensor to record the values.
    isr_1_StartEx(Timer_ISR_Handler_Front_Left);
    isr_2_StartEx(Timer_ISR_Handler_Front_Right);
    isr_3_StartEx(Timer_ISR_Handler_Left);
    isr_4_StartEx(Timer_ISR_Handler_Right);

    UART_Start();
    
    // left motor startup
    LeftMotor_Encoder_ISR_ClearPending();
    LeftMotor_Encoder_ISR_StartEx(left_motor_encoder);
    PWM_LeftMotor_1_Start();
    PWM_LeftMotor_2_Start();     
    
    // right motor startup
    RightMotor_Encoder_ISR_ClearPending();
    RightMotor_Encoder_ISR_StartEx(right_motor_encoder);
    PWM_RightMotor_1_Start();
    PWM_RightMotor_2_Start();
    
    LED_Write(1);
}
void show_step(int i){
    while (i){
        LED_Write(1);
        CyDelay(200);
        LED_Write(0);
        CyDelay(200);
        i = i-1;
    }
}
int puck_count = 0;

//==========================================================================
int main(void)
{
    
    InitializeHardware(); /* Enable global interrupts. */
    open_grippers_wide();
    raise();
    LED_Write(0);
    
    int button_pressed = 1;
    while (button_pressed == 1) {
        button_pressed = Start_Button_Read();
    }
    
    get_color_order();
    //show_step(1);
    int k = look_for_block();
    //show_step(2);
    position_after_block_search(k);
    //show_step(3);
    int puck_position, position;
    if (k == 4 || k == 0) {
        position = 0;
        puck_position = look_for_pucks(0); // robot is on the left side of arena
    } else {
        position = 1;
        puck_position = look_for_pucks(1); // robot is on the right side of arena
    } 
    //show_step(4);
    int color;
    color = pick_up_puck(position, puck_position, 0);
    while (color < 0) {
        color = pick_up_puck(position, puck_position, 1);
    }
    //return_with_puck(position, color);
    //show_step(5);
    stack_puck(position);
    color = pick_up_puck(position, puck_position, 0);
    while (color < 0) {
        color = pick_up_puck(position, puck_position, 1);
    }
    //return_with_puck(position, color);
    //show_step(6);
    stack_puck(position);
    if (position == 0) {
        color = pick_up_puck(position, puck_position, 0);
        while (color < 0) {
            color = pick_up_puck(position, puck_position, 1);
        }
        //return_with_puck(position, color);
        //show_step(7);
        stack_puck(position);
    }
//    get_color_order();
//    int k = look_for_block();
//    position_after_block_search(k);
//    int p;
//    if (k == 4 || k == 0) {
//        p = look_for_pucks(0); // robot is on the left side of arena
//    } else {
//        p = look_for_pucks(1); // robot is on the right side of arena
//    }
    
    //pick_up_puck(0, 3, 0);
    for(;;)
    {   
          LED_Write(1);
//        lower();
//        open_grippers();
//        CyDelay(2000);
//        close_grippers();
//        raise();
//        int x = color_test2();
//        if (x!=3){
//            puck_order[puck_count] = x;
//            puck_count++;
//        }
//        if(puck_count == 3){
//            break;
//        }
//        LED_Write(0);
//        CyDelay(2000);
//        int i;
//        for (int i = 0; i < p; i++){
//            LED_Write(1);
//            CyDelay(200);
//            LED_Write(0);
//            CyDelay(400);
//        }
        //sprintf(buf, "complted: left = %f, right: %f\n", value, value2);
        //UART_1_PutString(buf);
        
    }
    
//    for (int i = 0; i < puck_count; i++){
//        sprintf(buf, "%s\n", color_str[puck_order[i]]);
//        UART_PutString(buf);
//    }
}
//==========================================================================

void rotate(float angle, int direction, int speed) {
    reset_motor_counts();
    stop_motors();
    CyDelay(100);
    
    // going by time now
    speed = 100;
    float time_factor;
    if (direction == 0) time_factor = 24.55;
    if (direction == 1) time_factor = 24.55;
    int time_needed = (int) (angle * time_factor);
    
    max_lmc = 200000; //Not using counts
    max_rmc = 200000;
    
    
//    int counts_per_revolution = 1220;
//    float travel_per_revolution = 0.1772; // travel in meters
//    float distance_between_wheels = 0.225; //distance between the wheels
//    float counts_needed = (angle/360) * (M_PI * distance_between_wheels) / travel_per_revolution * counts_per_revolution;
//    
//    left_motor_count = 0;
//    right_motor_count = 0;
//    max_lmc = counts_needed; //PWM stop after count reached
//    max_rmc = counts_needed;
//    stop_all_motors = 1;
//    motors_running = 1;
    
    if (direction == 1) { // turning left
        PWM_LeftMotor_2_WriteCompare(speed);
        PWM_RightMotor_1_WriteCompare(speed);
    } else { // turning right
        PWM_LeftMotor_1_WriteCompare(speed);
        PWM_RightMotor_2_WriteCompare(speed);
    }
    CyDelay(time_needed);
    stop_motors();
//    while(motors_running) {
//        CyDelay(100);
//    }
    reset_motor_counts();
}

void pivot(int angle, int direction, int pivot_wheel, int speed) {
    int counts_per_revolution = 895;
    float travel_per_revolution = 0.1759; // travel in meters
    float distance_between_wheels = 0.225; //distance between the wheels
    float counts_needed = (angle/360) * (2* M_PI * distance_between_wheels) / travel_per_revolution * counts_per_revolution;
    
    
    left_motor_count = 0;
    right_motor_count = 0;
    max_lmc = counts_needed; //PWM stop after count reached
    max_rmc = counts_needed;
    stop_all_motors = 1;
    motors_running = 1;
    
    if (direction == 1) { // turning left
        if (pivot_wheel == 1) { // pivot on left wheel
            PWM_RightMotor_1_WriteCompare(speed);
        } else { // pivot on right wheel
            PWM_LeftMotor_2_WriteCompare(speed);
        }
    } else { // turning right
        if (pivot_wheel == 1) { // pivot on left wheel
            PWM_RightMotor_2_WriteCompare(speed);
        } else { // pivot on right wheel
            PWM_LeftMotor_1_WriteCompare(speed);
        }
    }
    
    while(motors_running) 
    {
        CyDelay(100);
    }
    reset_motor_counts();
}

void stabilized_move(float distance, int direction, int speed, float wall_distance) {
    stop_motors();
    reset_motor_counts();
    CyDelay(200);
    int counts_per_revolution = 895;
    float travel_per_revolution = 0.1759; // travel in meters
    float counts_needed = distance / travel_per_revolution * counts_per_revolution;
    float front_us;
    
    left_motor_count = 0;
    right_motor_count = 0;
    max_lmc = counts_needed; //PWM stop after count reached
    max_rmc = counts_needed;
    stop_all_motors = 1;
    motors_running = 1;

    // moving fowards dont care about direction parameter for prelim
    move_forward(speed);
    while(motors_running) {
        front_us = Front_Left_Ultrasonic_Sensor(200, 1);
        if (front_us < wall_distance) { // dont bump into anything
            stop_motors();
            front_us = Front_Right_Ultrasonic_Sensor(200, 1); // extra check
            if (front_us < wall_distance) {
                front_us = Front_Right_Ultrasonic_Sensor(1300, 1); // extra check
                if (front_us < wall_distance) {
                    motors_running = 0;
                    break;
                } else {
                    move_forward(speed);
                }
            } else {
                move_forward(speed);
            }
        }
    }
    reset_motor_counts();
}

// returns 0 if found during initial scan (block on right side)
// returns 1 if found during initial scan (block on left side)
// returns 2 if found once turned towrds back wall (block on left side)
// returns 3 if found during travel towards back wall (block on left side)
// returns 4 if not found and reached end of travel towards back wall (block on right side)
int look_for_block() {
    
    rotate(20.0, 1, 70);
    move_forward(120);
    CyDelay(2500);
    stop_motors();
    rotate(20.0, 0, 70);
    reset_motor_counts();
    int speed = 150;
    float distance = 0.53; // travel 84cm forward
    int counts_per_revolution = 895;
    float travel_per_revolution = 0.1759; // travel in meters
    float counts_needed = distance / travel_per_revolution * counts_per_revolution;
    max_lmc = counts_needed;
    max_rmc = counts_needed;
    stop_all_motors = 1;
    motors_running = 1;

    move_forward(speed);
    
    while(motors_running) {  
            
        float right_distance = Right_Ultrasonic_Sensor(150, 0);
        
        if (right_distance < 0.22) { // determine this value to be more accurate
            //CyDelay(200); // keep moving forward a little bit to avoid ultrasonic glitchy spots
            stop_motors();
            right_distance = Right_Ultrasonic_Sensor(2000, 1); // make sure this is where the block starts
            if (right_distance < 0.22) {
                int return_val = 1;
                
                if (left_motor_count < (0.15/travel_per_revolution*counts_per_revolution)){
                        return_val = 0;
                }
                
                // wait for robot to reach wall so more room to manouvre 
                move_forward(speed);
                while(motors_running) {
                    // wait to reach wall
                }
                return return_val; // block encountered during initial run
            } else {
                move_forward(speed);
            }
        }
    }
    
    // turning right
    stop_motors();
    reset_motor_counts();
    align(2);
    rotate(90.0, 1, 50);
    reset_motor_counts();
    
    if (Front_Left_Ultrasonic_Sensor(1000, 1) < 0.23 || Front_Right_Ultrasonic_Sensor(1000, 1) < 0.23) return 2; // block on far left side of arena  
    
    distance = 0.475; // travel 50cm forward
    counts_needed = distance / travel_per_revolution * counts_per_revolution;
    max_lmc = counts_needed;
    max_rmc = counts_needed;
    stop_all_motors = 1;
    motors_running = 1;
    move_forward(speed);
    
    while (motors_running) {
//        float left_distance = Left_Ultrasonic_Sensor(200, 0);
        
//        if (left_distance < 0.04) { // determine this value to be more accurate
//            CyDelay(200); // keep moving forward a little bit to avoid ultrasonic glitchy spots
//            stop_motors();
//            left_distance = Left_Ultrasonic_Sensor(500, 1); // make sure this is where the block is
//            if (left_distance < 0.04) {
//                return 3; // block is on the far left side of the arena
//            } else {
//                move_forward(speed);
//            }
//        }
    }
    
    return 4; //if it hasn't found the block on the left side it must be on the right side
}


void align(int count) {
    if (count <= 0) return;
    // save current counts to restore later
    int lmc_copy, rmc_copy, max_lmc_copy, max_rmc_copy;
    lmc_copy = left_motor_count;
    rmc_copy = right_motor_count;
    max_lmc_copy = max_lmc;
    max_rmc_copy = max_rmc;
    
    
    // ensure intterupt won't kill PWM signal
    max_lmc = 100000; 
    max_rmc = 100000;
    
    // stop motors
    stop_motors(); // stop motors while checking ultrasonics
    
    
    int speed = 25; // slow speed
    float left = Front_Left_Ultrasonic_Sensor(700, 1);
    float right = Front_Right_Ultrasonic_Sensor(700, 1);
    float init_difference = left - right; // left us - right us
    if (fabs(init_difference) < 0.005) return;
    
    if (init_difference < 0) { // left further from wall, turn right to correct
        PWM_LeftMotor_1_WriteCompare(speed);
        PWM_RightMotor_2_WriteCompare(speed);
    
    } else { // right further from the wall, turn left to correct
        PWM_LeftMotor_2_WriteCompare(speed);
        PWM_RightMotor_1_WriteCompare(speed);
    }
    

    int i, max_iterations = 200;
    float min_difference = 200000;
    int min_time = 0;
    int sample_time = 60;
    for (i=0; i<max_iterations; i++) {
        
        float difference = Front_Left_Ultrasonic_Sensor(sample_time, 0) - Front_Right_Ultrasonic_Sensor(sample_time, 0);

        if (fabs(difference) < min_difference) {
            min_difference = fabs(difference);
            min_time = 0;
        } else {
            min_time += sample_time * 2;
            if (min_time >= sample_time * 8) { // minimum needs to be minimum for 4 successive samples to be aligned
                break;
            }
        }
    }
    stop_motors();
    if (init_difference < 0) { // turning back after overturning
        PWM_LeftMotor_2_WriteCompare(speed);
        PWM_RightMotor_1_WriteCompare(speed);
    
    } else { // right further from the wall, turn left to correct
        PWM_LeftMotor_1_WriteCompare(speed);
        PWM_RightMotor_2_WriteCompare(speed);
    }
    CyDelay(min_time+sample_time);
        
    // stop motors
    stop_motors(); 
    
    // restore counts
    left_motor_count = lmc_copy;
    right_motor_count = rmc_copy;
    max_lmc = max_lmc_copy;
    max_rmc = max_rmc_copy;
    
    align(count-1); // repeat until difference small
    
}


float Front_Left_Ultrasonic_Sensor(int sample_time, int use_median)
{   
    us_sum_front_left = 0;
    us_counts_front_left = 0;
    int time_taken = 0;

    //Front_Left Ultrasonic Sensor
    while (time_taken < sample_time)
    {
            int current_counts = us_counts_front_left;
            Trigger_Write(1);
            CyDelayUs(10);
            Trigger_Write(0);
            
            int breakout = 0;
            while (us_counts_front_left == current_counts) {
                if (breakout >= 10) break; // no response in 10ms
                CyDelay(1);
                time_taken += 1;
                breakout++;
            }
            
    }
    if (use_median == 0) {
        return us_sum_front_left / (float)us_counts_front_left; // average
    } else {
        return calculate_median_distance(us_values_front_left, us_counts_front_left);
    }
}

float Front_Right_Ultrasonic_Sensor (int sample_time, int use_median)
{
    us_sum_front_right = 0;
    us_counts_front_right = 0;
    int time_taken = 0;

    //Front_Right Ultrasonic Sensor
    while (time_taken < sample_time)
    {
            int current_counts = us_counts_front_right;
            Trigger_1_Write(1);
            CyDelayUs(10);
            Trigger_1_Write(0);
            
            int breakout = 0;
            while (us_counts_front_right == current_counts) {
                if (breakout >= 10) break; // no response in 10ms
                CyDelay(1);
                time_taken += 1;
                breakout++;
            }
            
    }
    if (use_median == 0) {
        return us_sum_front_right / (float)us_counts_front_right; // average
    } else {
        return calculate_median_distance(us_values_front_right, us_counts_front_right);
    }
}

float Left_Ultrasonic_Sensor(int sample_time, int use_median)
{
    us_sum_left = 0;
    us_counts_left = 0;
    int time_taken = 0;

    //Front_Right Ultrasonic Sensor
    while (time_taken < sample_time)
    {
            int current_counts = us_counts_left;
            Trigger_2_Write(1);
            CyDelayUs(10);
            Trigger_2_Write(0);
            
            int breakout = 0;
            while (us_counts_left == current_counts) {
                if (breakout >= 10) break; // no response in 10ms
                CyDelay(1);
                time_taken += 1;
                breakout++;
            }
            
    }
    
    if (use_median == 0) {
        return us_sum_left / (float)us_counts_left; // average
    } else {
        return calculate_median_distance(us_values_left, us_counts_left);
    }
}

float Right_Ultrasonic_Sensor(int sample_time, int use_median)
{
    us_sum_right = 0;
    us_counts_right = 0;
    int time_taken = 0;

    //Front_Right Ultrasonic Sensor
    while (time_taken < sample_time)
    {
            int current_counts = us_counts_right;
            Trigger_3_Write(1);
            CyDelayUs(10);
            Trigger_3_Write(0);
            
            int breakout = 0;
            while (us_counts_right == current_counts) {
                if (breakout >= 10) break; // no response in 10ms
                CyDelay(1);
                time_taken += 1;
                breakout++;
            }
            
    }
    
    if (use_median == 0) {
        return us_sum_right / (float)us_counts_right; // average
    } else {
        return calculate_median_distance(us_values_right, us_counts_right);
    }
}

float Side_Ultrasonic_Sensor(int sample_time) 
{
    if (side == 1) {
        return Left_Ultrasonic_Sensor(sample_time, 0);
    } else {
        return Right_Ultrasonic_Sensor(sample_time, 0);
    }
}

float calculate_median_distance(float * distance_array, int counts){
    int i, j;
    int median = 0; // position in distance array
    int below_match = (int) ((float) counts / 2.0);
    for (i=0; i<counts; i++) {
        int below = 0;
        for (j=0; j<counts; j++) {
            if (distance_array[j] < distance_array[i]) below++;
        }
        if (below == below_match) {
            median = i;
            break;
        }
    }
    
    float distance_sum = 0.0;
    int sum_counts = 0;
    for (i=0; i<counts; i++) {
        if (fabs(distance_array[median] - distance_array[i]) < 0.03) { // within 3cm of the median
            distance_sum += distance_array[i];
            sum_counts++;
        }
    }
    
    return distance_sum / sum_counts;
}

void stop_motors() {
    // stop motors
    PWM_LeftMotor_1_WriteCompare(0); 
    PWM_RightMotor_1_WriteCompare(0);
    PWM_LeftMotor_2_WriteCompare(0);
    PWM_RightMotor_2_WriteCompare(0);
}

void move_forward(int speed) {
    stop_motors();
    PWM_LeftMotor_2_WriteCompare(speed);
    PWM_RightMotor_2_WriteCompare(speed);
}

void move_backward(int speed) {
    stop_motors();
    PWM_LeftMotor_1_WriteCompare(speed);
    PWM_RightMotor_1_WriteCompare(speed);
}

void reset_motor_counts(){
    left_motor_count = 0;
    right_motor_count = 0;
    max_lmc = 200000;
    max_rmc = 200000;
}

void get_color_order() {
    int speed = 90;
    int puck_time = 1750/2; // time taken to go to next puck holder
    // move to 5th holder
    reset_motor_counts();
    move_forward(speed);
    CyDelay(puck_time);
    stop_motors();
    
    int current_puck_1 = 2;
    int current_puck_holder;
    for (current_puck_holder = 0; current_puck_holder < 5; current_puck_holder++){
        if (current_puck_1 >= 0) {
            int color = color_test1(); // get color on side color sensor
            if (color != 3) { // puck holder not empty
                puck_order[current_puck_1] = color;
                puck_colors[color] += 1;
                current_puck_1--;
            }
        }
        move_backward(speed);
        CyDelay(puck_time);
        stop_motors();
    }
    if (puck_order[1] < 0) {
            puck_order[1] = puck_order[2];
    }
    if (puck_order[1] < 0) {
            puck_order[1] = 1;
            puck_order[0] = 2;
    } else {
        int k;
        for (k=1; k<3; k++) {
            if (puck_order[0] < 0) {
                puck_order[0] = puck_order[k];
            }
        }
    }
    
    move_backward(speed);
    CyDelay(puck_time);
    stop_motors();
}

/////   ?????????????????????????????????????

void position_after_block_search(int location) {
    reset_motor_counts();
    if (location == 4) return; //already in position
    if (location == 0) {
        align(2);
        rotate(90.0, 1, 70);
        
        stabilized_move(0.475,1,150,0.0);
        
        return;
    }
    if (location == 3) {
        rotate(180.0, 1, 70);
        stabilized_move(80.0,1,150,0.15);
        align(2);
        rotate(90.0, 0, 70);
    } else if (location == 2) {
        rotate(90.0, 1, 70);
    } else if (location == 1) {
//        rotate(10.0, 1, 70);
//        reset_motor_counts();
//        CyDelay(200);
//        move_forward(70);
//        CyDelay(1500);  // avoiding front puck holder
//        stop_motors();
//        reset_motor_counts();
//        rotate(170.0, 1, 70);
        rotate(180.0, 1, 70);
    }
    stabilized_move(80.0,1,150,0.20);
    align(2);
    rotate(90.0, 0, 70);
    stabilized_move(0.465,1,150,0.0);
}

// checks if pucks are in front of the robot or not, i.e whether pucks should be picked up from the side or front of the puck array
// position = 0 (left side of arena), position = 1 (right side)
int look_for_pucks(int position) {
    stabilized_move(0.22, 1, 150, 0.035); // move forward to be near pucks
    float left_distance = Front_Left_Ultrasonic_Sensor(500, 1);
    float right_distance = Front_Right_Ultrasonic_Sensor(500, 1);
    
    rotate(10, 1, 120);
    float left_distance_1 = Front_Left_Ultrasonic_Sensor(500, 1);
    float right_distance_1 = Front_Right_Ultrasonic_Sensor(500, 1);
    
    rotate(20, 0, 120);
    float left_distance_2 = Front_Left_Ultrasonic_Sensor(500, 1);
    float right_distance_2 = Front_Right_Ultrasonic_Sensor(500, 1);
    rotate(10, 1, 120);
    
    int pucks_on_right = right_distance < 0.15 || right_distance_1 < 0.12 || right_distance_2 < 0.12;
    int pucks_on_left = left_distance < 0.15 || left_distance_1 < 0.12 || left_distance_2 < 0.12;
    if (position == 0) { // robot on left side
        if (pucks_on_left && !pucks_on_right) { // pucks on left wall
            return 0;
        } else if (pucks_on_left) { // pucks directly in front of robot
            return 1;
        } else if (pucks_on_right) { // pucks slightly away from wall
            return 2;
        } else {
            return 3;
        }
    } else { // robot on right side
        if (pucks_on_right && !pucks_on_left) { // pucks on right wall
            return 0;
        } else if (pucks_on_right) { // pucks directly in front of robot
            return 1;
        } else if (pucks_on_left) { // pucks slightly away from wall
            return 2;
        } else {
            return 3;
        }
    }
    
}

// position = 0 (left side of arena), position = 1 (right_side)
// puck position = 0 (pucks on wall), 1 (pucks directly in front), 2 (pucks slightly away from wall), 3 (pucks away from wall)
// repeat = 0 when this is called for a new puck, 1 when this is called after a failed puck pickup
// returns -1 if no puck picked up, 0 = R, 1 = G, 2 = B

int pick_up_puck(int position, int puck_position, int repeat) {
    int depth = 0; // 0 for first layer, 1 for second etc
    int steps = 0; // how many pucks away from inital puck this one is
    if (position == 0) {
        if (puck_position == 0) {
            if (repeat == 0) { // position closer to wall
                rotate(30.0, 1, 70);
                move_backward(100);
                CyDelay(1550);
                stop_motors();
                rotate(30.0, 0, 70);
                stabilized_move(0.18, 1, 150, 0);
            }
            
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth); // grabs a puck
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                rotate(90.0, 1, 70);
                // dump puck
                move_forward(70);
                CyDelay(2000);
                stop_motors();
                open_grippers();
                close_grippers();
                move_backward(70);
                CyDelay(2000);
                stop_motors();
                rotate(90.0, 0, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                rotate(180.0, 1, 70);
                pucks_picked += 1;
                puck_colors[color_check_result] -= 1;
                return color_check_result;
            }
         
        } else if (puck_position == 1) {
            // same as 0 
            // dump pucks closer as well
            
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            
            // return to original position
            
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth);
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                rotate(90.0, 1, 70);
                // dump puck
                move_forward(70);
                CyDelay(1000);
                stop_motors();
                open_grippers();
                close_grippers();
                move_backward(70);
                CyDelay(1000);
                stop_motors();
                rotate(90.0, 0, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                rotate(90.0, 0, 70);
                stabilized_move(10, 1, 150, 0.18);
                align(2);
                rotate(90.0, 0, 70);
                return color_check_result;
            }
            
        } else if (puck_position == 2) {
            if (repeat == 0) { // position closer to wall
                rotate(20,0,70);
                move_backward(120);
                CyDelay(1500);
                stop_motors();
                rotate(20,1,70);
                stabilized_move(0.18, 1, 150, 0);
            }
            
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth); // grabs a puck
            
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                rotate(90.0, 1, 70);
                // dump puck
                lower();
                open_grippers();
                raise();
                close_grippers();
                rotate(90.0, 0, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                rotate(90.0, 0, 70);
                stabilized_move(10, 1, 150, 0.18);
                align(2);
                rotate(90.0, 0, 70);
                pucks_picked += 1;
                puck_colors[color_check_result] -= 1;
                return color_check_result;
            }
        } else if (puck_position == 3) {
            if (repeat == 0) {
                // position robot closer to left wall
                rotate(30.0, 1, 70);
                move_backward(100);
                CyDelay(1550);
                stop_motors();
                rotate(30.0, 0, 70);
                stabilized_move(0.8, 1, 120, 0.14); // move to 10cm away from far wall
                align(2);
                rotate(60.0, 1, 70);
                move_forward(100);
                CyDelay(1200);
                rotate(30.0, 1, 70); 
                if (distance_from_wall == 0) { // distance not set, have to use ultrasonics
                    reset_motor_counts();
                    move_forward(100);
                    while(1) {
                        float right_distance = Front_Right_Ultrasonic_Sensor(100, 0);
                        float left_distance = Front_Left_Ultrasonic_Sensor(100, 0);
            
                        if (right_distance < 0.07) { // less than 7cm
                            //CyDelay(100); // keep moving forward a little bit to avoid ultrasonic glitchy spots
                            stop_motors();
                            right_distance = Front_Right_Ultrasonic_Sensor(1000, 1); // make sure this is where the pucks are
                            if (right_distance < 0.07) {
                                distance_from_wall = right_motor_count;
                                break;
                            } else {
                                move_forward(70);
                            }
                        }
                        
                        if (left_distance < 0.07) { // less than 7cm
                            //CyDelay(100); // keep moving forward a little bit to avoid ultrasonic glitchy spots
                            stop_motors();
                            left_distance = Front_Left_Ultrasonic_Sensor(1000, 1); // make sure this is where the pucks are
                            if (left_distance < 0.07) {
                                distance_from_wall = left_motor_count;
                                break;
                            } else {
                                move_forward(70);
                            }
                        }
                    }   
                } else { // distance set, use this instead of ultrasonics
                    reset_motor_counts();
                    max_lmc = distance_from_wall;
                    max_rmc = distance_from_wall;
                    move_forward(120);
                    motors_running = 1;
                    stop_all_motors = 1;
                    int max_delay = 15000;
                    int current_delay = 0;
                    while (motors_running && current_delay<max_delay) {
                        CyDelay(100);
                        current_delay += 100;
                        // pass the time :)
                    }  
                }
            }
            
            // now 5cm from the pucks
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            
            // return to original position
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth);
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                // dump puck
                rotate(90.0, 1, 120);
                lower();
                open_grippers();
                raise();
                close_grippers();
                rotate(90.0, 0, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                
                
//                
//                rotate(30.0, 1, 70); 
//                move_backward(100);
//                CyDelay(1200);
//                rotate(60.0, 1, 70);
//                
                
                rotate(30.0, 0, 120);
                move_backward(100);
                CyDelay(1500);
                stop_motors();
                rotate(30.0, 1, 120);
                move_forward(100);
                CyDelay(1200);
                stop_motors();
                rotate(90.0, 0, 70);
                align(1);
                rotate(90.0, 0, 120);
                stabilized_move(10, 1, 130, 0.18);
                align(2);
                rotate(90.0, 0, 70);
                pucks_picked += 1;
                return color_check_result;
            }
            
        }
        
        
    } else if (position == 1) {
        if (puck_position == 0) {
            if (repeat == 0) { // position closer to wall
                rotate(30.0, 0, 70);
                move_backward(100);
                CyDelay(1550);
                stop_motors();
                rotate(30.0, 1, 70);
                stabilized_move(0.18, 1, 150, 0);
            }
            
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth); // grabs a puck
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                rotate(90.0, 0, 70);
                // dump puck
                move_forward(70);
                CyDelay(2000);
                stop_motors();
                open_grippers();
                close_grippers();
                move_backward(70);
                CyDelay(2000);
                stop_motors();
                rotate(90.0, 1, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                rotate(180.0, 0, 70);
                pucks_picked += 1;
                puck_colors[color_check_result] -= 1;
                return color_check_result;
            }
         
        } else if (puck_position == 1) {
            // same as 0 
            // dump pucks closer as well
            
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            
            // return to original position
            
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth);
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                rotate(90.0, 0, 70);
                // dump puck
                move_forward(70);
                CyDelay(1000);
                stop_motors();
                open_grippers();
                close_grippers();
                move_backward(70);
                CyDelay(1000);
                stop_motors();
                rotate(90.0, 1, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                rotate(90.0, 1, 70);
                stabilized_move(10, 1, 150, 0.18);
                align(2);
                rotate(90.0, 1, 70);
                return color_check_result;
            }
            
        } else if (puck_position == 2) {
            if (repeat == 0) { // position closer to wall
                rotate(20,1,70);
                move_backward(120);
                CyDelay(1500);
                stop_motors();
                rotate(20,0,70);
                stabilized_move(0.18, 1, 150, 0);
            }
            
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth); // grabs a puck
            
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                rotate(90.0, 0, 70);
                // dump puck
                lower();
                open_grippers();
                raise();
                close_grippers();
                rotate(90.0, 1, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                rotate(90.0, 1, 70);
                stabilized_move(10, 1, 150, 0.18);
                align(2);
                rotate(90.0, 1, 70);
                pucks_picked += 1;
                puck_colors[color_check_result] -= 1;
                return color_check_result;
            }
        } else if (puck_position == 3) {
            if (repeat == 0) {
                // position robot closer to left wall
                rotate(30.0, 0, 70);
                move_backward(100);
                CyDelay(1900);
                stop_motors();
                rotate(30.0, 1, 70);
                stabilized_move(0.8, 1, 120, 0.14); // move to 10cm away from far wall
                align(2);
                rotate(60.0, 0, 70);
                move_forward(100);
                CyDelay(1200);
                rotate(30.0, 0, 70); 
                if (distance_from_wall == 0) { // distance not set, have to use ultrasonics
                    reset_motor_counts();
                    move_forward(100);
                    while(1) {
                        float right_distance = Front_Right_Ultrasonic_Sensor(100, 0);
                        float left_distance = Front_Left_Ultrasonic_Sensor(100, 0);
            
                        if (right_distance < 0.07) { // less than 7cm
                            //CyDelay(100); // keep moving forward a little bit to avoid ultrasonic glitchy spots
                            stop_motors();
                            right_distance = Front_Right_Ultrasonic_Sensor(1000, 1); // make sure this is where the pucks are
                            if (right_distance < 0.07) {
                                distance_from_wall = right_motor_count;
                                break;
                            } else {
                                move_forward(70);
                            }
                        }
                        
                        if (left_distance < 0.07) { // less than 7cm
                            //CyDelay(100); // keep moving forward a little bit to avoid ultrasonic glitchy spots
                            stop_motors();
                            left_distance = Front_Left_Ultrasonic_Sensor(1000, 1); // make sure this is where the pucks are
                            if (left_distance < 0.07) {
                                distance_from_wall = left_motor_count;
                                break;
                            } else {
                                move_forward(70);
                            }
                        }
                    }   
                } else { // distance set, use this instead of ultrasonics
                    reset_motor_counts();
                    max_lmc = distance_from_wall;
                    max_rmc = distance_from_wall;
                    move_forward(120);
                    motors_running = 1;
                    stop_all_motors = 1;
                    int max_delay = 15000;
                    int current_delay = 0;
                    while (motors_running && current_delay<max_delay) {
                        CyDelay(100);
                        current_delay += 100;
                        // pass the time :)
                    }  
                }
            }
            
            // now 5cm from the pucks
            steps = pucks_picked % 3;
            depth = pucks_picked / 3; // 0 for first layer, 1 for second etc
            
            
            // return to original position
            int color_check_result = 3;
            color_check_result = pick_puck(steps, depth);
            
            if (color_check_result == 3) { // no puck in gripper
                pucks_picked += 1;
                return -1;
            }
            
            if (puck_colors[color_check_result]  <= 0) { // color not needed
                // dump puck
                rotate(90.0, 0, 120);
                lower();
                open_grippers();
                raise();
                close_grippers();
                rotate(90.0, 1, 70);
                pucks_picked += 1;
                return -1;
            } else {     //color needed
                
                rotate(30.0, 1, 120);
                move_backward(100);
                CyDelay(1500);
                stop_motors();
                rotate(30.0, 0, 120);
                move_forward(100);
                CyDelay(1200);
                stop_motors();
                rotate(90.0, 1, 70);
                align(1);
                rotate(90.0, 1, 120);
                stabilized_move(10, 1, 130, 0.18);
                align(2);
                rotate(90.0, 1, 70);
                pucks_picked += 1;
                return color_check_result;
            }
            
        }
        
     
    }
    return -1;
}

int get_stack_position(int color) {
    int i;
    for (i = 0; i<3; i++){
        if (puck_order[i] == color && stash_positions[i] == 0) {
            return i;
        }
    }
    return i;
}

// arena_position 0= left side, 1 = right side
// puck_position = 0 (pucks on wall), 1 (pucks directly in front), 2 (pucks slightly away from wall), 3 (pucks away from wall)
// color = 0 R, 1 G, 2 B
void return_with_puck(int arena_position, int color) {
    if (arena_position == 0) {
        
        stabilized_move(10, 1, 150, 0.18);
        align(2);
        
        // putting puck in right place for later retrieval
        // reverse for certain dist depending on what order the puck is in stack
        int stack_position = get_stack_position(color);
        if (stack_position == 0) {
            rotate(90.0, 0, 120);
            stash_positions[0] = 1;
        } else if (stack_position == 1) {
            rotate(70.0, 0, 120);
            stash_positions[1] = 1;
        } else if (stack_position == 2) {
            rotate(110.0, 0, 120);
            stash_positions[2] = 1;
        }
        lower();
        open_grippers();
        raise();
        close_grippers();
        if (stack_position == 0) {
            rotate(90.0, 1, 120);
        } else if (stack_position == 1) {
            rotate(70, 1, 120);
        } else if (stack_position == 2) {
            rotate(110.0, 1, 120);
        }
        
        // return to midway position
        align(2);
        
        if (current_puck != 2) {
            rotate(180.0, 1, 70);
            stabilized_move(0.6, 1, 150, 0);  // determine this value!
        } else {
            stack_pucks(0);
        }
        
    } else if (arena_position == 1) {
        
        stabilized_move(10, 1, 150, 0.18);
        align(2);
        rotate(90.0, 1, 120);
        stabilized_move(10, 1, 150, 0.15);
        align(2);
        
        // putting puck in right place for later retrieval
        // reverse for certain dist depending on what order the puck is in stack
        int stack_position = get_stack_position(color);
        if (stack_position == 0) {
            rotate(90.0, 1, 120);
            stash_positions[0] = 1;
        } else if (stack_position == 1) {
            rotate(70.0, 1, 120);
            stash_positions[1] = 1;
        } else if (stack_position == 2) {
            rotate(110.0, 1, 120);
            stash_positions[2] = 1;
        }
        lower();
        open_grippers();
        raise();
        close_grippers();
        if (stack_position == 0) {
            rotate(90.0, 0, 120);
        } else if (stack_position == 1) {
            rotate(70, 0, 120);
        } else if (stack_position == 2) {
            rotate(110.0, 0, 120);
        }
        
        // return to midway position
        align(2);
        
        if (current_puck != 2) {
            rotate(180.0, 1, 70);
            stabilized_move(10, 1, 150, 0.15);  // determine this value!
            align(2);
            rotate(90.0, 0, 120);
            stabilized_move(0.5, 1, 150, 0);
        } else {
            stack_pucks(1);
        }
    }
    
    current_puck += 1;
}


void stack_pucks(int position) {
    open_grippers_wide();
    if (position == 0) {
        rotate(90.0, 1, 120);
        align(2);
        move_backward(100);
        CyDelay(500);
        stop_motors();
        stabilized_move(10, 1, 90, 0.12);
        rotate(90.0, 0, 120);
    } else {
        align(2);
        move_backward(100);
        CyDelay(500);
        stop_motors();
        stabilized_move(10, 1, 90, 0.12);
    }
    int i;
    for (i=0; i<3; i++) {
        if (position == 0) {
            int forward_time = 1500;
            int back_time = 500;
            float rotate_angle;
            if (i == 0) rotate_angle = 90.0;
            if (i == 1) rotate_angle = 70.0;
            if (i == 2) rotate_angle = 110.0;
            rotate(rotate_angle, 0, 100);
            open_grippers();
            move_backward(100);
            CyDelay(back_time);
            stop_motors();
            lower();
            move_forward(100);
            CyDelay(forward_time);
            stop_motors();
            close_grippers();
            raise();
            move_backward(100);
            CyDelay(forward_time-back_time);
            stop_motors();
            rotate(rotate_angle, 1, 100);
            lower_lv(i);
            open_grippers();
            raise_lv();
        } else {
            int forward_time = 1500;
            int back_time = 500;
            float rotate_angle;
            if (i == 0) rotate_angle = 90.0;
            if (i == 1) rotate_angle = 70.0;
            if (i == 2) rotate_angle = 110.0;
            rotate(rotate_angle, 1, 100);
            open_grippers();
            move_backward(100);
            CyDelay(back_time);
            stop_motors();
            lower();
            move_forward(100);
            CyDelay(forward_time);
            stop_motors();
            close_grippers();
            raise();
            move_backward(100);
            CyDelay(forward_time-back_time);
            stop_motors();
            rotate(rotate_angle + 90.0, 0, 100);
            lower_lv(i);  
            open_grippers();
            raise_lv();
            if (i < 2){
                rotate(90.0, 1, 100);
            }
        }
    }
    
    move_backward(120);
    CyDelay(1000);
    stop_motors();
    rotate(90.0, 1, 120);
    align(2);
    
    // move to home
    float front_distance = Front_Right_Ultrasonic_Sensor(2000, 1);
    float distance = 0.45 - front_distance;  
    int counts_per_revolution = 895;
    float travel_per_revolution = 0.1759; // travel in meters
    float counts_needed = distance / travel_per_revolution * counts_per_revolution;
    left_motor_count = 0;
    right_motor_count = 0;
    max_lmc = counts_needed; //PWM stop after count reached
    max_rmc = counts_needed;
    stop_all_motors = 1;
    motors_running = 1;
    move_backward(70);
    while(motors_running) {
        // do nothing
    }
    
    rotate(60.0, 1, 70);
    move_backward(80);
    CyDelay(3000);
    stop_motors();
    rotate(60.0, 0, 70);
    move_backward(100);
    CyDelay(2000);
    stop_motors();
}


int pick_puck(int steps, int depth) {
    
    stop_motors();
    reset_motor_counts();
    if (steps == 0) {} // no rotate needed
    if (steps == 1) rotate(14, 1, 100);
    if (steps == 2) rotate(14, 0, 100);
    move_backward(120);
    CyDelay(1000);
    stop_motors();
    lower();
    open_grippers();
    move_forward(120);
    CyDelay(1000);
    stop_motors();
    
    move_forward(85);
    CyDelay(500 + depth*800); // travel time to reach first layer, plus any additional layers required
    stop_motors();
    close_grippers();
    raise();
    int color = color_test2();
    if (puck_order[current_puck] != color && puck_colors[color]  > 0) {
        lower();
        open_grippers();
        raise();
    } 
    move_backward(85);
    CyDelay(500 + depth*800);
    stop_motors();
    
    if (steps == 0) {} // no rotate needed
    if (steps == 1) rotate(14, 0, 100);
    if (steps == 2) rotate(14, 1, 100);
    if (puck_order[current_puck] != color && puck_colors[color]  > 0) {
        return 3;
    } else {
        return color;
    }
}


void stack_puck(int arena_position) {
    int pucks_to_stack = 2;
    if (arena_position == 0) {
        pucks_to_stack = 3;
        stabilized_move(10, 1, 150, 0.18);
        rotate(90.0, 1, 120);
        align(3);
        
        move_backward(100);
        CyDelay(1000);
        stop_motors();
        stabilized_move(10, 1, 80, 0.135);
        rotate(80.0, 0, 120);
        move_backward(100);
        CyDelay(1000);
        stop_motors();
        stabilized_move(10, 1, 80, 0.14);
        lower_lv(current_puck);
        CyDelay(200);
        open_grippers();
        raise_lv();
        
        // return to midway position
        
        if (current_puck != pucks_to_stack-1) {
            open_grippers_wide();
            move_backward(120);
            CyDelay(500);
            stop_motors();
            
            rotate(80.0, 1, 120);
            align(1);
            move_backward(120);
            CyDelay(500);
            stop_motors();
            
            rotate(90.0, 1, 70);
            close_grippers();
            stabilized_move(0.56, 1, 150, 0);  // determine this value!
        } else {
                open_grippers_wide();
                move_backward(120);
                CyDelay(1000);
                stop_motors();
                rotate(90.0, 1, 120);
                align(2);
                
                // move to home
                float front_distance = Front_Right_Ultrasonic_Sensor(2000, 1);
                float distance = 0.43 - front_distance;  
                int counts_per_revolution = 895;
                float travel_per_revolution = 0.1759; // travel in meters
                float counts_needed = distance / travel_per_revolution * counts_per_revolution;
                left_motor_count = 0;
                right_motor_count = 0;
                max_lmc = counts_needed; //PWM stop after count reached
                max_rmc = counts_needed;
                stop_all_motors = 1;
                motors_running = 1;
                move_backward(70);
                while(motors_running) {
                    // do nothing
                }
                
                rotate(60.0, 1, 70);
                move_backward(80);
                CyDelay(3200);
                stop_motors();
                rotate(60.0, 0, 70);
                move_backward(100);
                CyDelay(750);
                stop_motors();
        }
        
    } else if (arena_position == 1) {
        
                
        stabilized_move(10, 1, 150, 0.18);
        align(4);
        rotate(90.0, 1, 120);
        stabilized_move(10, 1, 150, 0.18);
        align(3);
        
        move_backward(100);
        CyDelay(1000);
        stop_motors();
        stabilized_move(10, 1, 100, 0.135);
        rotate(80.0, 0, 120);
        move_backward(100);
        CyDelay(1000);
        stop_motors();
        stabilized_move(10, 1, 100, 0.14);
        lower_lv(current_puck);
        CyDelay(200);
        open_grippers();
        raise_lv();
        
        // return to midway position
        
        if (current_puck != pucks_to_stack-1) {
            open_grippers_wide();
            move_backward(120);
            CyDelay(500);
            stop_motors();
            rotate(80.0, 1, 70);
            align(1);
            rotate(180, 1, 120);
            
            stabilized_move(10, 1, 150, 0.2);
            align(2);
            rotate(90.0, 0, 120);
            stabilized_move(0.56, 1, 150, 0);  // determine this value!
        } else {
                move_backward(120);
                CyDelay(1000);
                stop_motors();
                rotate(90.0, 1, 120);
                align(2);
                
                // move to home
                float front_distance = Front_Right_Ultrasonic_Sensor(2000, 1);
                float distance = 0.43 - front_distance;  
                int counts_per_revolution = 895;
                float travel_per_revolution = 0.1759; // travel in meters
                float counts_needed = distance / travel_per_revolution * counts_per_revolution;
                left_motor_count = 0;
                right_motor_count = 0;
                max_lmc = counts_needed; //PWM stop after count reached
                max_rmc = counts_needed;
                stop_all_motors = 1;
                motors_running = 1;
                move_backward(70);
                while(motors_running) {
                    // do nothing
                }
                
                rotate(60.0, 1, 70);
                move_backward(80);
                CyDelay(3200);
                stop_motors();
                rotate(60.0, 0, 70);
                move_backward(100);
                CyDelay(750);
                stop_motors();
        }
       
    }
    
    current_puck += 1;
    
}
/* [] END OF FILE */