#ifndef FWDOG_H_INCLUDED
#define FWDOG_H_INCLUDED


#ifdef __cplusplus
extern "C" {
#endif

#define TASK_GNSS_COMM2_BIT ( 1 << 0 )
#define TASK_GNSS_COMM3_BIT ( 1 << 1 )
#define TASK_IMU_BIT 		( 1 << 2 )
#define ALL_SYNC_BITS 		( TASK_GNSS_COMM2_BIT | TASK_GNSS_COMM3_BIT | TASK_IMU_BIT )

void fwdog_init(void);
void fwdog_feed(void);
void fwdog_task_create(void);


#ifdef __cplusplus
}
#endif

#endif // gd32F20X_40X_SPI_H_INCLUDED



