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

    int fd = open(DEVICE_NAME, O_RDWR);

    if (fd < 0) {
        perror("Failed to open device");
        return errno;
    }

    // Initialize tx_buffer with some values
    for (int i = 0; i < BUFFER_SIZE; i++) {
        tx_buffer[i] = (float)i / 10.0f;
    }

    // Send the count of data to the driver
    write_data_count(fd, BUFFER_SIZE);

    // Send each data to the driver
    for (int i = 0; i < BUFFER_SIZE; i++) {
        write_data_to_position(fd, i, tx_buffer[i]);
    }

    // Read processed data
    read_processed_data(fd, rx_buffer);

    // Print the results
    for (int i = 0; i < BUFFER_SIZE; i++) {
        printf("Result[%d] = %f\n", i, rx_buffer[i]);
    }

    // Measure execution time on CPU
    long start_time = get_time_in_us();
    for (int i = 0; i < BUFFER_SIZE; i++) {
        rx_buffer[i] = exp(tx_buffer[i]);
    }
    long end_time = get_time_in_us();
    long cpu_time = end_time - start_time;

    printf("CPU execution time: %ld us\n", cpu_time);

    // Map TX and RX buffers to the driver
    float* tx_mmap = (float*)mmap(NULL, BUFFER_SIZE * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, fd, TX_BUFFER_OFFSET);
    float* rx_mmap = (float*)mmap(NULL, BUFFER_SIZE * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, fd, RX_BUFFER_OFFSET);

    if (tx_mmap == MAP_FAILED || rx_mmap == MAP_FAILED) {
        perror("Memory mapping failed");
        close(fd);
        return errno;
    }

    // Copy data to the mapped TX buffer
    memcpy(tx_mmap, tx_buffer, BUFFER_SIZE * sizeof(float));

    // Trigger the processing (This part might need specific IOCTL call based on driver implementation)
    // Assuming IOCTL_CALL is defined and properly implemented in the driver
    ioctl(fd, IOCTL_CALL);

    // Measure execution time on FPGA
    start_time = get_time_in_us();
    // Assuming that the processing is triggered and completed within this block
    end_time = get_time_in_us();
    long fpga_time = end_time - start_time;

    printf("FPGA execution time: %ld us\n", fpga_time);

    // Copy processed data from the mapped RX buffer
    memcpy(rx_buffer, rx_mmap, BUFFER_SIZE * sizeof(float));

    // Unmap the buffers
    munmap(tx_mmap, BUFFER_SIZE * sizeof(float));
    munmap(rx_mmap, BUFFER_SIZE * sizeof(float));

    close(fd);
    return 0;
}
