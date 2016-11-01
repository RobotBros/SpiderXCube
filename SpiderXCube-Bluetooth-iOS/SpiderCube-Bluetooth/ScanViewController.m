//
//  ScanViewController.m
//  SpiderCube-Bluetooth
//
//  Created by Enix Yu on 21/4/16.
//  Copyright © 2016 RobotBros. All rights reserved.
//

#import <MBProgressHUD.h>
#import "ScanViewController.h"
#import "BTManager.h"
#import "CharacteristicViewController.h"


static NSString *kPeripheralCell = @"peripheralCell";

@interface ScanViewController ()

@property (nonatomic, strong) BTManager *btManager;
@property (weak, nonatomic) IBOutlet UIBarButtonItem *scanButton;
@property (nonatomic, strong) MBProgressHUD *hud;

@end

#pragma mark - Lifecycle

@implementation ScanViewController

- (void)viewDidLoad {
    [super viewDidLoad];

    _btManager = [BTManager sharedInstance];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    
    [[UIApplication sharedApplication] sendAction:_scanButton.action to:_scanButton.target from:nil forEvent:nil];
}

#pragma mark - UITableViewDataSource

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    return [[_btManager peripheralsFound] count];
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:kPeripheralCell];
    NSDictionary *peripheral = [[_btManager peripheralsFound] objectAtIndex:indexPath.row];
    cell.textLabel.text = [peripheral objectForKey:@"name"];
    return cell;
}

#pragma mark - UITableViewDelegate

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    NSIndexPath *indexPath = [self.tableView indexPathForCell:sender];
    CharacteristicViewController *incomingVc = [segue destinationViewController];
    
    NSDictionary *peripheral = [[_btManager peripheralsFound] objectAtIndex:indexPath.row];
    [incomingVc setPeripheralName:[peripheral objectForKey:@"name"]];
    [incomingVc setTitle:[peripheral objectForKey:@"name"]];
}

- (IBAction)scanButtonDidClicked:(id)sender
{
    if ([_btManager isAvailable]){
        
        if (!_btManager.isScanning) {
            
            UIBarButtonItem *button = [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemStop target:self action:@selector(scanButtonDidClicked:)];
            self.navigationItem.rightBarButtonItem = button;

            _hud = [MBProgressHUD showHUDAddedTo:self.view animated:YES];
            _hud.labelText = NSLocalizedString(@"Scanning peripheral...", @"Scanning");
            
            [_btManager scanPeripheralWithCompletion:^(NSError *error) {
                dispatch_async(dispatch_get_main_queue(), ^{
                    [_hud hide:YES];
                    [self.tableView reloadData];
                });
            }];
        } else {
            
            [_btManager stopScan];

            dispatch_async(dispatch_get_main_queue(), ^{
                UIBarButtonItem *button = [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemRefresh target:self action:@selector(scanButtonDidClicked:)];
                self.navigationItem.rightBarButtonItem = button;
                [_hud hide:YES];
            });
        }
    } else {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:NSLocalizedString(@"Error", @"Error") message:NSLocalizedString(@"Bluetooth 4.0 is not available in your device", @"Not available") delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil, nil];
        [alert show];
    }
}


@end
