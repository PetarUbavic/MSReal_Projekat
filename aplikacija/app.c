#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>

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
void write_data_count(int fd, int count) {
    char command[20];
    sprintf(command, "Unesite broj clanova niza: %d\n", count);
    write(fd, command, strlen(command));
}

// Function to write data to specific positions
void write_data_to_position(int fd, int position, float number) {
    char command[40];
    sprintf(command, "Pozicija=%08x", *(unsigned int*)&number);
    lseek(fd, position * sizeof(float), SEEK_SET);
    write(fd, command, strlen(command));
}

// Function to read processed data
void read_processed_data(int fd, float* buffer) {
    char result[2048];
    lseek(fd, 0, SEEK_SET);
    read(fd, result, sizeof(result));
    char* token = strtok(result, ",");
    int index = 0;
    while (token != NULL && index < BUFFER_SIZE) {
        sscanf(token, "%08x", (unsigned int*)&buffer[index]);
        token = strtok(NULL, ",");
        index++;
    }
}

int main() {

    unsigned int array_num = 0;
    float value = 0;
    

label1:    printf("Unesite broj clanova niza: ");
    scanf("%d", &array_num);
    printf("\n");
    printf("Uneli ste %d clanova niza\n", array_num);

    if(array_num > 256 || array_num < 1) {
        printf("Pogresan unos\n")
        goto label1;
    }

    float tx_buffer[array_num];
    float rx_buffer[array_num];
    float rx_buffer_cpu[array_num];

    if (array_num > 254) {

        for(int i = 0; i < array_num; i++) {
            tx_buffer[i] = 7;    // zato sto se dobija skoro ceo broj
        }
    }

    else {

        for(int i = 0; i < array_num; i++) {
            printf("Unesite clan niza na %d poziciji: ", i);
            scanf("%d", &value);
            printf("\n");
            tx_buffer[i] = value;
            printf("Na poziciji %d nalazi se vrednost: %f\n", i, tx_buffer[i]);
        }
    }

    tx_buffer[array_num] = '\0';


    int fd = open(DEVICE_NAME, O_RDWR | O_NDELAY);

    if (fd < 0) {
        printf("Failed to open device\n");
        return errno;
    }

    // Send the data to the device
    ssize_t bytes_written = write(fd, tx_buffer, sizeof(float) * array_num);

    if (bytes_written < 0) {
        printf("Failed to write to device\n");
        close(fd);
        return errno;
    } 
    
    else {
        printf("Successfully wrote %zd bytes to the device\n", bytes_written);
    }

    // Read processed data
    read_processed_data(fd, rx_buffer_cpu);

    // Print the results
    for (int i = 0; i < array_num; i++) {
        printf("Result[%d] = %f\n", i, rx_buffer_cpu[i]);
    }

    // Measure execution time on CPU
    long start_time = get_time_in_us();
    for (int i = 0; i < array_num; i++) {
        rx_buffer_cpu[i] = exp(tx_buffer[i]);
    }
    long end_time = get_time_in_us();
    long cpu_time = end_time - start_time;

    printf("CPU execution time: %ld us\n", cpu_time);

    // Map TX and RX buffers to the driver
    float* tx_mmap = (float*)mmap(NULL, array_num * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, fd, TX_BUFFER_OFFSET);
    float* rx_mmap = (float*)mmap(NULL, array_num * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, fd, RX_BUFFER_OFFSET);

    if (tx_mmap == MAP_FAILED || rx_mmap == MAP_FAILED) {
        printf("Memory mapping failed\n");
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
    return 0;
}
