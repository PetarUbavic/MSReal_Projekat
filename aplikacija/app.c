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
#define BUFFER_SIZE 4096            // sa 256 prilikom iscitavanja moze da se procita samo 21 clan niza, ovako moze svih 256 clanova niza 
#define TX_BUFFER_OFFSET 0x00
#define RX_BUFFER_OFFSET 0x2048

#define MMAP

uint floatToHex(float num) {
	uint newNum;
	if(num == 0) {
		newNum = 0x0;
		return newNum;
	}

	int sign = (num < 0) ? 1: 0;
	int exp = 0;
	float normalizedValue = fabs(num);
	while(normalizedValue >= 2.0) {
		normalizedValue /= 2.0;
		exp++;	
	}
	while(normalizedValue < 1.0) {
		normalizedValue *= 2.0;
		exp--;	
	}
	exp += 127;
	
	uint mantissa = (uint)((normalizedValue - 1.0) * pow(2, 23));
	
	newNum = (sign << 31) | (exp << 23) | mantissa;
	return newNum;
}


float hexToFloat(uint number) {
	float newNumber;
	uint sign = (number >> 31) & 0x1; 
	uint exp = (number >> 23) & 0xFF; 
	uint mantissa = number & 0x7FFFFF;
	exp -= 127;
	
	uint exponentValue = 1;
    uint i = 1;
	for(i = 1; i <= exp; i++) {
		exponentValue *= 2;
	}
	float mantissaValue = 1.0 + ((float)mantissa / pow(2, 23));
	newNumber = exponentValue  * mantissaValue;
	if(sign == 1) {
		newNumber *= -1;
	}
	return newNumber;	
}

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
int write_data_to_position(int fd, int position, uint number) {
    char command[40];
    sprintf(command, "Pozicija=%d=0x%08x", position, *(unsigned int*)&number);
    lseek(fd, position * sizeof(uint), SEEK_SET);

    if ((write(fd, command, strlen(command))) < 0) {
        return -1;
    }

    else {
        return 0;
    }
}

int write_start_command(int fd) {
    char command[20];
    sprintf(command, "START");

    if (write(fd, command, strlen(command)) < 0) {
        return -1; // Return -1 if the write operation fails
    } else {
        return 0; // Return 0 on success
    }
}



int main() {

    unsigned int array_num = 0;
    float value = 0;
    uint hex_value = 0;
    int i = 0;
    int count = 0;
    int ret = 0;
    long start_time = 0;
    long end_time = 0;
    long fpga_time = 0;
    long cpu_time = 0;

    FILE *fp;
    int fd;
    char path[BUFFER_SIZE];
    

label1:    printf("Unesite broj - clanova niza: ");
    scanf("%d", &array_num);
    printf("Uneli ste %d clanova niza\n", array_num);

    if(array_num > 256 || array_num < 1) {
        printf("Pogresan unos\n");
        goto label1;
    }

    uint tx_buffer[array_num];
    uint rx_buffer[array_num];
    float rx_buffer_cpu[array_num];

    if (array_num == 256) {

        for(i = 0; i < array_num; i++) {
            tx_buffer[i] = floatToHex(1);
        }
    }

    else {

        for(i = 0; i < array_num; i++) {
            printf("Unesite clan niza na %d poziciji: ", i);
            scanf("%f", &value);
            //printf("\n");
            hex_value = floatToHex(value);
            tx_buffer[i] = hex_value;
        }
    }

    tx_buffer[array_num] = '\0';


    fd = open(DEVICE_NAME, O_RDWR);

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

    #ifndef MMAP

    for(i = 0; i < array_num; i++) {

        fd = open(DEVICE_NAME, O_RDWR);

        if (fd < 0) {
            printf("Failed to open device\n");
            return errno;
        }

        ret = write_data_to_position(fd, i, tx_buffer[i]);

        if(ret < 0) {
            printf("Failed to write %#x to position %d\n", tx_buffer[i], i);
            close(fd);
            return errno;
        }
        else {
            printf("Successfully wrote %#x to position %d\n", tx_buffer[i], i);
        }

        close(fd);
    }

    // Read processed data
    // Open the command for reading
    fp = popen("cat /dev/fpu_exp", "r");
    if (fp == NULL) {
        printf("Failed to run command\n");
        exit(1);
    }

    // Read the output
    if (fgets(path, sizeof(path), fp) != NULL) {
        // printf("Raw output: %s\n", path);  // Debugging line to show raw output

        // Tokenize the string using ',' as the delimiter
        char *token = strtok(path, ", ");
        while (token != NULL && count < 256) {
           // printf("Token: %s\n", token);  // Debugging line to show each token
            // Convert the hex string to unsigned int and store it in the array
            if (sscanf(token, "0x%x", &rx_buffer[count]) == 1) {
                count++;
            } else {
                printf("Failed to parse token: %s\n", token);  // Debugging line
            }
            token = strtok(NULL, ", ");
        }
    } else {
        printf("Failed to read from the command output\n");
    }
    // Close the pipe
    pclose(fp);

    // Print the values for verification
/*    printf("Received hex numbers:\n");
    for (i = 0; i < count; i++) {
        printf("%#x\n", rx_buffer[i]);
    }

    printf("Received float numbers:\n");
    for (i = 0; i < count; i++) {
        if((double)hexToFloat((float)rx_buffer[i]) == 0) {                          // isinf((double)hexToFloat((float)rx_buffer[i])) ovo ne radi, pa cu proveriti da li je (double)hexToFloat((float)rx_buffer[i]) == 0, posto e stepenovano bilo kojim realnim brojem nije 0, onda ako je 0 znaci da je rezultat beskonacno
            printf("Inf\n");
        }
        else {
            printf("%f\n", hexToFloat((float)rx_buffer[i]));
        }
    }
*/
    #else

    fd = open(DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        printf("Failed to open device\n");
        return errno;
    }

    // Map TX and RX buffers to the driver
    uint* tx_mmap = (uint*)mmap(0, array_num * sizeof(uint), PROT_READ | PROT_WRITE, MAP_SHARED, fd, TX_BUFFER_OFFSET);

    if (tx_mmap == MAP_FAILED) {
        printf("Memory mapping TX failed\n");
        close(fd);
        return errno;
    }

    // Copy data to the mapped TX buffer
    memcpy(tx_mmap, tx_buffer, array_num * sizeof(uint));

    
    // Measure execution time on FPGA
    start_time = get_time_in_us();
    
    // Trigger the processing
    ret = write_start_command(fd);

    // Copy processed data from the mapped TX buffer
    memcpy(rx_buffer, tx_mmap, array_num * sizeof(uint));

    end_time = get_time_in_us();
    fpga_time = end_time - start_time;    
    
    // Unmap the buffers
    munmap(tx_mmap, array_num * sizeof(uint));
    close(fd);

    #endif

    // Measure execution time on CPU
    start_time = get_time_in_us();
    for (i = 0; i < array_num; i++) {
        rx_buffer_cpu[i] = exp(1);      // ne idemo sa hexToFloat(tx_buffer[i]), kako bi merili samo trajanje exp f-je, a ne i hexToFloat f-je
    }
    end_time = get_time_in_us();
    cpu_time = end_time - start_time;
    

    // Print the results
    for (i = 0; i < array_num; i++) {
         if((double)hexToFloat((float)rx_buffer[i]) == 0) {
            printf("FPGA Result[%d] = Inf\n", i);
        }
        else {
            printf("FPGA Result[%d] = %f\n", i, hexToFloat(rx_buffer[i]));
        }
    }

    printf("FPGA execution time: %ld us\n", fpga_time);
    printf("CPU execution time: %ld us\n", cpu_time);

    return 0;
}
