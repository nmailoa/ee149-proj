
/*
 
 Copyright (c) 2013-2014 RedBearLab
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 
 */

#import "RBLAppDelegate.h"

@implementation RBLAppDelegate

@synthesize  ble;

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    // Insert code here to initialize your application
    [self.textView setEditable:NO];
    
    ble = [[BLE alloc] init];
    [ble controlSetup];
    ble.delegate = self;
}

- (BOOL) applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)application
{
    return YES;
}

-(void) bleDidConnect
{
    NSLog(@"->Connected");
    
    btnConnect.title = @"Disconnect";
    [indConnect stopAnimation:self];
    
    [ble readRSSI];
}

- (void)bleDidDisconnect
{
    NSLog(@"->Disconnected");
    
    btnConnect.title = @"Connect";
    
    lblRSSI.stringValue = @"RSSI: -127";
}

-(void) bleDidReceiveData:(unsigned char *)data length:(int)length
{
    NSLog(@"Length: %d", length);

    data[length] = 0;
    NSString *str = [NSString stringWithCString:data encoding:NSUTF8StringEncoding];
    
    
    NSLog(@"%@", str);
    
    static NSMutableString *message;
    static NSMutableString *last_message;
    
    if (message == nil)
        message = [[NSMutableString alloc] initWithString:@""];
    last_message = [[NSMutableString alloc] initWithString:@""];

    [message appendString:str];
    [message appendString:@"\n"];
    [last_message appendString:str];
    [last_message appendString:@"\n"];
    
    self.textView.string = message;
    [self.textView scrollRangeToVisible: NSMakeRange(self.textView.string.length, 0)];
    
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory,NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *documentTXTPath = [documentsDirectory stringByAppendingPathComponent:@"reWRITEData.csv"];
    //NSString *savedString = textview.text;
    NSFileManager *fileManager = [NSFileManager defaultManager];
    if(![fileManager fileExistsAtPath:documentTXTPath])
    {
        [last_message writeToFile:documentTXTPath atomically:YES];
    }
    else
    {
        NSFileHandle *myHandle = [NSFileHandle fileHandleForWritingAtPath:documentTXTPath];
        [myHandle seekToEndOfFile];
        [myHandle writeData:[last_message dataUsingEncoding:NSUTF8StringEncoding]];
    }
}

-(void) bleDidUpdateRSSI:(NSNumber *) rssi
{
    lblRSSI.stringValue = [NSString stringWithFormat:@"RSSI: %@", rssi.stringValue];
}

- (IBAction)btnConnect:(id)sender
{
    
    NSLog(@"Clicked");
    
    
    if (ble.activePeripheral)
        if(ble.activePeripheral.isConnected)
        {
            [[ble CM] cancelPeripheralConnection:[ble activePeripheral]];
            return;
        }
    
    if (ble.peripherals)
        ble.peripherals = nil;
    
    [ble findBLEPeripherals:2];
    
    [NSTimer scheduledTimerWithTimeInterval:(float)2.0 target:self selector:@selector(connectionTimer:) userInfo:nil repeats:NO];
    
    [indConnect startAnimation:self];
}

-(void) connectionTimer:(NSTimer *)timer
{
    if (ble.peripherals.count > 0)
    {
        for (int i = 0; i < ble.peripherals.count; i++) {
            CBPeripheral *p = [ble.peripherals objectAtIndex:i];
            
            if ([p.name rangeOfString:@"Simple Chat"].location != NSNotFound) {
                NSLog(@"Found Simple Chat!");
                [ble connectPeripheral:[ble.peripherals objectAtIndex:i]];
            }
        }
        
    }
    else
    {
        [indConnect stopAnimation:self];
    }
}

-(IBAction)sendTextOut:(id)sender
{
    UInt8 buf[20];
    [text.stringValue getCString:buf maxLength:20 encoding:NSUTF8StringEncoding];
    
    NSData *data = [[NSData alloc] initWithBytes:buf length:text.stringValue.length];
    [ble write:data];
}

@end
