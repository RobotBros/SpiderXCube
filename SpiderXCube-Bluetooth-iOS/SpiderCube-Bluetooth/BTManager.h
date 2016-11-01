//
//  BTManager.h
//  SpiderCube-Bluetooth
//
//  Created by Enix Yu on 21/4/16.
//  Copyright © 2016 RobotBros. All rights reserved.
//

#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>

typedef void (^BTScanComplete)(NSError *error);
typedef void (^BTConnectComplete)(NSError *error);
typedef void (^BTReadComplete)(NSData *data, NSError *error);

@interface BTManager : NSObject

@property (nonatomic, assign, readonly, getter=isAvailable) BOOL available;
@property (nonatomic, assign, readonly, getter=isScanning) BOOL scanning;
@property (nonatomic, assign, readonly, getter=isConnecting) BOOL connecting;
@property (nonatomic, assign, readonly, getter=isConnected) BOOL connected;

@property (nonatomic, strong, readonly) CBPeripheral *connectedPeripheral;

/**
 * @brief  Get singleton object of BTManager
 */
+ (instancetype)sharedInstance;

/**
 * @brief  Scan peripheral
 */
- (void)scanPeripheralWithCompletion:(BTScanComplete)completion;

/**
 * @brief  Stop scan peripheral
 */
- (void)stopScan;

/**
 * @brief  Connect peripheral
 * @param  name  The name of peripheral to be connected.
 */
- (void)connectPeripheralWithName:(NSString *)name complete:(BTConnectComplete)completion;

/**
 * @brief  Disconnect peripheral
 */
- (void)disconnect;

/**
 * @brief  Read data from characteristic
 * @param  characteristicUUID  The characteristic UUID
 * @param  completion   The callback when read is done
 */
- (void)readCharacteristic:(NSString *)characteristicUUID completion:(BTReadComplete)completion;

/**
 * @brief  Write data to characteristic
 * @param  characteristicUUID  The characteristic UUID
 * @param  data   The data to be written to characteristic
 */
- (void)writeCharacteristic:(NSString *)characteristicUUID withData:(NSData *)data;

/**
 * @brief  Subscribe or unscribe to characteristic
 * @param  isNotify  TRUE - Subscribe; FALSE - Unscribe
 * @param  characteristicUUID  The characteristic UUID
 * @param  callback  The callback for characteristic value changed
 */
- (void)setNotify:(BOOL)isNotify forCharacteristic:(NSString *)characteristicUUID withCallback:(BTReadComplete)callback;

/**
 * @brief  Get the array of peripherals found by the central
 */
- (NSArray *)peripheralsFound;

@end
