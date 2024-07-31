#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <math.h>

#define DEVICE_NAME "/dev/fpu_exp"
#define BUFFER_SIZE 256
#define TX_BUFFER_OFFSET 0x00
#define RX_BUFFER_OFFSET 0x1000

// Function to get the current time in microseconds
static long get_time_in_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

// Function to write the number of data to be processed
int write_data_count(int fd, int count) {
    char command[20];
    sprintf(command, "N=%d", count);

    if ((write(fd, command, strlen(command))) < 0) {
        return -1;
    }

    else {
        return 0;
    }
}

// Function to write data to specific positions
int write_data_to_position(int fd, int position, float number) {
    char command[40];
    sprintf(command, "Pozicija=%d=0x%08x", position, *(unsigned int*)&number);
    lseek(fd, position * sizeof(float), SEEK_SET);

    if ((write(fd, command, strlen(command))) < 0) {
        return -1;
    }

    else {
        return 0;
    }
}

// Function to read processed data
/*
void read_processed_data(int fd, float* buffer) {
    char result[2048];
    lseek(fd, 0, SEEK_SET);
    read(fd, result, sizeof(result));
    char* token = strtok(result, ",");
    index = 0;
    while (token != NULL && index < BUFFER_SIZE) {
        sscanf(token, "%08x", (unsigned int*)&buffer[index]);
        token = strtok(NULL, ",");
        index++;
    }
}
*/

int main() {

    unsigned int array_num = 0;
    float value = 0;
    int i = 0;
    int ret = 0;
    

label1:    printf("Unesite broj clanova niza: ");
    scanf("%d", &array_num);
    printf("Uneli ste %d clanova niza\n", array_num);

    if(array_num > 256 || array_num < 1) {
        printf("Pogresan unos\n");
        goto label1;
    }

    float tx_buffer[array_num];
    float rx_buffer[array_num];
    float rx_buffer_cpu[array_num];

    if (array_num > 254) {

        for(i = 0; i < array_num; i++) {
            tx_buffer[i] = 7;    // zato sto se dobija skoro ceo broj
        }
    }

    else {

        for(i = 0; i < array_num; i++) {
            printf("Unesite clan niza na %d poziciji: ", i);
            scanf("%f", &value);
            //printf("\n");
            tx_buffer[i] = value;
            printf("Na poziciji %d nalazi se vrednost: %f\n", i, tx_buffer[i]);
        }
    }

    tx_buffer[array_num] = '\0';


    int fd = open(DEVICE_NAME, O_RDWR);

    if (fd < 0) {
        printf("Failed to open device\n");
        return errno;
    }

    // Send the data to the device
    ret = write_data_count(fd, array_num);

    if (ret < 0) {
        printf("Failed to write to device\n");
        close(fd);
        return errno;
    } 
    else {
        printf("Successfully wrote %zd to the device\n", array_num);
    }

    close(fd);

    for(i = 0; i < array_num; i++) {

        fd = open(DEVICE_NAME, O_RDWR);

        if (fd < 0) {
            printf("Failed to open device\n");
            return errno;
        }

        ret = write_data_to_position(fd, i, tx_buffer[i]);

        if(ret < 0) {
            printf("Failed to write %f to position %d\n", tx_buffer[i], i);
            close(fd);
            return errno;
        }
        else {
            printf("Successfully wrote %f to position %d\n", tx_buffer[i], i);
        }

        close(fd);
    }
    // Read processed data
    //read_processed_data(fd, rx_buffer_cpu);


    // Measure execution time on CPU
    long start_time = get_time_in_us();
    for (i = 0; i < array_num; i++) {
        rx_buffer_cpu[i] = exp(tx_buffer[i]);
    }
    long end_time = get_time_in_us();
    long cpu_time = end_time - start_time;

    // Print the results
    for (i = 0; i < array_num; i++) {
        printf("Result[%d] = %f\n", i, rx_buffer_cpu[i]);
    }

    printf("CPU execution time: %ld us\n", cpu_time);

    fd = open(DEVICE_NAME, O_RDWR);

    // Map TX and RX buffers to the driver
    float* tx_mmap = (float*)mmap(0, array_num * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, fd, TX_BUFFER_OFFSET);
    float* rx_mmap = (float*)mmap(NULL, array_num * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, fd, RX_BUFFER_OFFSET);

    if (tx_mmap == MAP_FAILED) {
        printf("Memory mapping TX failed\n");
        close(fd);
        return errno;
    }
    
    if (rx_mmap == MAP_FAILED) {
        printf("Memory mapping RX failed\n");
        close(fd);
        return errno;
    }

    // Copy data to the mapped TX buffer
    memcpy(tx_mmap, tx_buffer, array_num * sizeof(float));

    // Trigger the processing (This part might need specific IOCTL call based on driver implementation)
    // Assuming IOCTL_CALL is defined and properly implemented in the driver

    // Measure execution time on FPGA
    start_time = get_time_in_us();
    // Assuming that the processing is triggered and completed within this block
    end_time = get_time_in_us();
    long fpga_time = end_time - start_time;

    printf("FPGA execution time: %ld us\n", fpga_time);

    // Copy processed data from the mapped RX buffer
    memcpy(rx_buffer, rx_mmap, array_num * sizeof(float));

    // Unmap the buffers
    munmap(tx_mmap, array_num * sizeof(float));
    munmap(rx_mmap, array_num * sizeof(float));

    close(fd);

    // Print the results
    for (i = 0; i < array_num; i++) {
        printf("FPGA Result[%d] = %f\n", i, rx_buffer[i]);
    }

    return 0;
}
