//
//  CharacteristicViewController.m
//  SpiderCube-Bluetooth
//
//  Created by Enix Yu on 21/4/16.
//  Copyright © 2016 RobotBros. All rights reserved.
//

#import <MBProgressHUD.h>
#import "CharacteristicViewController.h"
#import "BTManager.h"
#import "ReadWriteViewController.h"

static NSString *const kCharacteristicCell = @"CharacteristicCell";

@interface CharacteristicViewController ()

@property (nonatomic, strong) BTManager *btManager;

@end

@implementation CharacteristicViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    _btManager = [BTManager sharedInstance];
    [self.navigationItem.leftBarButtonItem setTitle:nil];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (void)viewDidAppear:(BOOL)animated
{
    if ([_btManager isAvailable] && ![_btManager isScanning] && _peripheralName != nil) {
        MBProgressHUD *hud = [MBProgressHUD showHUDAddedTo:self.navigationController.view animated:YES];
        hud.labelText = NSLocalizedString(@"Connecting to peripheral...", @"Connecting...");
        [_btManager connectPeripheralWithName:_peripheralName complete:^(NSError *error) {
            
            dispatch_async(dispatch_get_main_queue(), ^{
                [hud hide:YES];
                
                if (error) {
                    UIAlertView *alert = [[UIAlertView alloc] initWithTitle:NSLocalizedString(@"Error", @"Error") message:NSLocalizedString(@"Can not connected to peripheral", @"error") delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil, nil];
                    [alert show];
                } else {
                    [self.tableView reloadData];
                }
            });
        }];
    } else {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:NSLocalizedString(@"Error", @"Error") message:NSLocalizedString(@"Internal Error", @"internal error") delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil, nil];
        [alert show];
    }
}

#pragma mark - Table view data source

- (NSString *)tableView:(UITableView *)tableView titleForHeaderInSection:(NSInteger)section
{
    CBService *serivice = [[[_btManager connectedPeripheral] services] objectAtIndex:section];
    return [[serivice UUID] UUIDString];
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
    return [[[_btManager connectedPeripheral] services] count];
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    CBService *serivice = [[[_btManager connectedPeripheral] services] objectAtIndex:section];
    return [[serivice characteristics] count];
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:kCharacteristicCell forIndexPath:indexPath];
    
    CBService *serivice = [[[_btManager connectedPeripheral] services] objectAtIndex:indexPath.section];
    CBCharacteristic *charac = [[serivice characteristics] objectAtIndex:indexPath.row];
    cell.textLabel.text = [[charac UUID] UUIDString];
    return cell;
}


#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    ReadWriteViewController *incomingVc = [segue destinationViewController];
    
    NSIndexPath *indexPath = [self.tableView indexPathForCell:sender];
    CBService *serivice = [[[_btManager connectedPeripheral] services] objectAtIndex:indexPath.section];
    CBCharacteristic *charac = [[serivice characteristics] objectAtIndex:indexPath.row];
    [incomingVc setCharacteristicUUID:[[charac UUID] UUIDString]];
}

@end
