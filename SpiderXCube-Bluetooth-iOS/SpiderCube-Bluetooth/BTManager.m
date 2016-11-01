//
//  BTManager.m
//  SpiderCube-Bluetooth
//
//  Created by Enix Yu on 21/4/16.
//  Copyright © 2016 RobotBros. All rights reserved.
//

#import "BTManager.h"


#define MASK_ENABLE(A, MASK)      (((A) & (MASK)) == (MASK))

@interface BTManager () <CBCentralManagerDelegate, CBPeripheralDelegate>

@property (nonatomic, assign, readwrite, getter=isAvailable) BOOL available;
@property (nonatomic, assign, readwrite, getter=isScanning) BOOL scanning;
@property (nonatomic, assign, readwrite, getter=isConnecting) BOOL connecting;
@property (nonatomic, assign, readwrite, getter=isConnected) BOOL connected;

@property (nonatomic, strong) CBCentralManager *centralManager;
@property (nonatomic, strong) NSMutableArray<NSDictionary *> *peripheralsFound;
@property (nonatomic, strong, readwrite) CBPeripheral *connectedPeripheral;

@property (nonatomic, strong) NSTimer *discoverTimer;

@property (nonatomic, strong) BTScanComplete scanCompletion;
@property (nonatomic, strong) BTConnectComplete connectedCompletion;
@property (nonatomic, strong) BTReadComplete readCallback;
@property (nonatomic, strong) BTReadComplete notifyCallback;


@end

@implementation BTManager

@synthesize peripheralsFound = _peripheralsFound;

#pragma mark - Lifecycle

- (instancetype)init
{
    self = [super init];
    if (self) {
        [self setAvailable:FALSE];
        [self setConnecting:FALSE];
        [self setConnected:FALSE];
        [self setScanning:FALSE];
        [self setPeripheralsFound:[NSMutableArray array]];
        dispatch_queue_t queue = dispatch_queue_create("cn.robotbros.spiderxcube", DISPATCH_QUEUE_CONCURRENT);
        [self setCentralManager:[[CBCentralManager alloc] initWithDelegate:self queue:queue]];
    }
    return self;
}

+ (instancetype)sharedInstance
{
    static BTManager *btManager = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        btManager = [[BTManager alloc] init];
    });
    return btManager;
}

- (void)discoverred:(NSTimer *)timer
{
    //
    // Discoverred finished;
    //
    if (_connectedCompletion) {
        _connected = TRUE;
        _connectedCompletion(nil);
    }
}

#pragma mark - Public methods

- (void)scanPeripheralWithCompletion:(BTScanComplete)completion;
{
    _scanning = TRUE;
    _scanCompletion = completion;
    [_centralManager scanForPeripheralsWithServices:nil options:@{CBCentralManagerScanOptionAllowDuplicatesKey: @YES}];
}

- (void)connectPeripheralWithName:(NSString *)name complete:(BTConnectComplete)completion;
{
    //
    // Connect periperal
    //
    _connecting = TRUE;
    _connectedCompletion = completion;

    for (NSDictionary *peripheral in _peripheralsFound) {
        if ([[peripheral objectForKey:@"name"] isEqualToString:name]){
            [_centralManager connectPeripheral:[peripheral objectForKey:@"cbperipheral"] options:nil];
            break;
        }
    }
}

- (void)disconnect
{
    _connected = FALSE;
    _connecting = FALSE;
    _scanning = FALSE;

    if (_connectedPeripheral){
        [_centralManager cancelPeripheralConnection:_connectedPeripheral];
    }
}

- (void)stopScan
{
    _scanning = FALSE;
}

- (void)readCharacteristic:(NSString *)characteristicUUID completion:(BTReadComplete)completion
{
    for (CBService *service in [_connectedPeripheral services]) {
        for (CBCharacteristic *charac in [service characteristics]) {
            if ([[[charac UUID] UUIDString] isEqualToString:characteristicUUID]) {
                if (MASK_ENABLE([charac properties], CBCharacteristicPropertyRead)) {
                    _readCallback = completion;
                    [_connectedPeripheral readValueForCharacteristic:charac];
                    break;
                }
            }
        }
    }
}

- (void)writeCharacteristic:(NSString *)characteristicUUID withData:(NSData *)data
{
    for (CBService *service in [_connectedPeripheral services]) {
        for (CBCharacteristic *charac in [service characteristics]) {
            if ([[[charac UUID] UUIDString] isEqualToString:characteristicUUID]) {
                if (MASK_ENABLE([charac properties], CBCharacteristicPropertyWrite | CBCharacteristicPropertyWriteWithoutResponse)) {
                    
                    [_connectedPeripheral writeValue:data forCharacteristic:charac type:CBCharacteristicWriteWithoutResponse];
                    break;
                }
            }
        }
    }
}

- (void)setNotify:(BOOL)isNotify forCharacteristic:(NSString *)characteristicUUID withCallback:(BTReadComplete)callback;
{
    for (CBService *service in [_connectedPeripheral services]) {
        for (CBCharacteristic *charac in [service characteristics]) {
            if ([[[charac UUID] UUIDString] isEqualToString:characteristicUUID]) {
                if (MASK_ENABLE([charac properties], CBCharacteristicPropertyNotify)) {
                    _readCallback = callback;
                    [_connectedPeripheral setNotifyValue:isNotify forCharacteristic:charac];
                    break;
                }
            }
        }
    }
}

#pragma mark - CBCentralManagerDelegate

- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
    switch (central.state) {
        case CBCentralManagerStatePoweredOff:
        case CBCentralManagerStateResetting:
        case CBCentralManagerStateUnauthorized:
        case CBCentralManagerStateUnknown:
        case CBCentralManagerStateUnsupported:
            [self setAvailable:FALSE];
            break;
        case CBCentralManagerStatePoweredOn:
            [self setAvailable:TRUE];
            break;
        default:
            break;
    }
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary<NSString *,id> *)advertisementData RSSI:(NSNumber *)RSSI
{
    //
    // Peripheral found
    //
    BOOL foundAlready = FALSE;
    for (NSDictionary *peripheralFound in _peripheralsFound) {
        if ([[peripheralFound objectForKey:@"name"] isEqualToString:peripheral.name]) {
            foundAlready = TRUE;
        }
    }
    
    if (!foundAlready){
        NSDictionary *peripheralFound = @{@"name": peripheral.name, @"cbperipheral": peripheral};
        [_peripheralsFound addObject:peripheralFound];
    } else {
        // Duplicate record found, stop scan
        _scanning = FALSE;
        [_centralManager stopScan];
        
        if (_scanCompletion){
            _scanCompletion(nil);
        }
    }
}

- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral
{
    _connectedPeripheral = peripheral;
    [peripheral setDelegate:self];
    [peripheral discoverServices:nil];
}

#pragma mark - CBPeripheralDelegate

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error
{
    if (error){
        if (_connectedCompletion){
            _connectedCompletion(error);
        }
    } else {
        
        dispatch_async(dispatch_get_main_queue(), ^{
            _discoverTimer = [NSTimer scheduledTimerWithTimeInterval:5.0
                                                              target:self
                                                            selector:@selector(discoverred:)
                                                            userInfo:nil
                                                             repeats:NO];
        });
        
        for (CBService *service in peripheral.services) {
            [peripheral discoverCharacteristics:nil forService:service];
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error
{
    dispatch_async(dispatch_get_main_queue(), ^{
        [_discoverTimer invalidate];
        _discoverTimer = [NSTimer scheduledTimerWithTimeInterval:5.0
                                                          target:self
                                                        selector:@selector(discoverred:)
                                                        userInfo:nil
                                                         repeats:NO];
    });

    if (error){
        if (_connectedCompletion){
            _connectedCompletion(error);
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
    //
    // read complete
    //
    if (_readCallback) {
        if (error) {
            _readCallback(nil, error);
        } else {
            _readCallback(characteristic.value, error);
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateNotificationStateForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
    //
    // notify
    //
    if (_readCallback) {
        if (error) {
            _readCallback(nil, error);
        } else {
            _readCallback(characteristic.value, error);
        }
    }
}

@end
