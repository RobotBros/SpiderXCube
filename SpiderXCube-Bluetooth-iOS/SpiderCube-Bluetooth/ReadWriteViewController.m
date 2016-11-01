//
//  ReadWriteViewController.m
//  SpiderCube-Bluetooth
//
//  Created by Enix Yu on 21/4/16.
//  Copyright © 2016 RobotBros. All rights reserved.
//

#import <MBProgressHUD.h>
#import "ReadWriteViewController.h"
#import "BTManager.h"

@interface ReadWriteViewController ()

@property (weak, nonatomic) IBOutlet UILabel *characteristicNameLabel;
@property (weak, nonatomic) IBOutlet UITextField *writeValueTextField;
@property (weak, nonatomic) IBOutlet UITextView *responseTextView;
@property (weak, nonatomic) IBOutlet UIButton *sendButton;
@property (weak, nonatomic) IBOutlet UIBarButtonItem *readButton;
@property (weak, nonatomic) IBOutlet UIButton *notifyButton;

@property (nonatomic, assign, getter=isNotify) BOOL notify;
@property (nonatomic, strong) BTReadComplete readCallback;

@property (nonatomic, strong) BTManager *btManager;

@end

@implementation ReadWriteViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    _btManager = [BTManager sharedInstance];
    _notify = FALSE;
    _characteristicNameLabel.text = _characteristicUUID;
    _responseTextView.layer.borderWidth = 1;
    _responseTextView.layer.borderColor = [UIColor grayColor].CGColor;
    _responseTextView.layer.cornerRadius = 3.0f;
    
    __weak typeof(self) weakSelf = self;
    _readCallback = ^(NSData *data, NSError *error){
        if (error){
            dispatch_async(dispatch_get_main_queue(), ^{
                UIAlertView *alert = [[UIAlertView alloc] initWithTitle:NSLocalizedString(@"Error", @"Error") message:NSLocalizedString(@"Failed to read", @"read failed") delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil, nil];
                [alert show];
            });
        } else {
            dispatch_async(dispatch_get_main_queue(), ^{
                NSMutableString *str = [NSMutableString stringWithString:weakSelf.responseTextView.text];
                [str appendString:[NSString stringWithFormat:@"%@\n", [weakSelf formatNSDataToHex:data]]];
                weakSelf.responseTextView.text = str;
            });
        }
    };
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

#pragma mark - Private methods

- (NSString *)formatNSDataToHex:(NSData *)data
{
    NSMutableString *str = [[NSMutableString alloc] init];
    
    for (int i = 0; i < data.length; i ++) {
        [str appendFormat:@"%02X ", ((Byte *)data.bytes)[i]];
    }
    
    return [str copy];
}

- (BOOL)isValidHexString:(NSString *)hexString
{
    NSError *error;
    NSRegularExpression *regex = [NSRegularExpression regularExpressionWithPattern:@"^[0-9A-Fa-f]*$" options:0 error:&error];
    
    if (hexString == nil || [hexString length] % 2 != 0 || ![regex numberOfMatchesInString:hexString options:0 range:NSMakeRange(0, [hexString length])]) {
        return FALSE;
    }
    
    return TRUE;
}

- (NSData *)stringHexToData:(NSString *)hexString
{
    if ([self isValidHexString:hexString]) {
        NSMutableData *data= [[NSMutableData alloc]init];
        unsigned char whole_byte;
        char byte_chars[3] = {'\0','\0','\0'};
        for (int i = 0; i < ([hexString length] / 2); i++)
        {
            byte_chars[0] = [hexString characterAtIndex:i*2];
            byte_chars[1] = [hexString characterAtIndex:i*2+1];
            whole_byte = strtol(byte_chars, NULL, 16);
            [data appendBytes:&whole_byte length:1];
        }
        return data;
    }
    
    return nil;
}

#pragma mark - IBAction

- (IBAction)readDidClicked:(id)sender
{
    if ([_btManager isConnected] && _characteristicUUID != nil) {
        [_btManager readCharacteristic:_characteristicUUID completion:_readCallback];
    } else {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:NSLocalizedString(@"Error", @"Error") message:NSLocalizedString(@"Internal Error", @"internal error") delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil, nil];
        [alert show];
    }
}

- (IBAction)sendDidClicked:(id)sender
{
    if ([_btManager isConnected] && _characteristicUUID != nil) {
        [_btManager writeCharacteristic:_characteristicUUID withData:[self stringHexToData:_writeValueTextField.text]];
    } else {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:NSLocalizedString(@"Error", @"Error") message:NSLocalizedString(@"Internal Error", @"internal error") delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil, nil];
        [alert show];
    }
    
    [_writeValueTextField resignFirstResponder];
}

- (IBAction)notifyDidClicked:(id)sender
{
    _notify = !_notify;
    if (_notify) {
        [_notifyButton setTitle:@"Un-notify" forState:UIControlStateNormal];
    } else {
        [_notifyButton setTitle:@"Notify" forState:UIControlStateNormal];
    }
    
    [_btManager setNotify:_notify forCharacteristic:_characteristicUUID withCallback:_readCallback];
}

- (IBAction)backgroundDidClicked:(id)sender
{
    [_writeValueTextField resignFirstResponder];
}

@end
