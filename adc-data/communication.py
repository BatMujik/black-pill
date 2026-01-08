import spidev
import time

class SpiRpcClient:
    def __init__(self, bus=0, device=0, max_speed_hz=1000000, debug=True):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = max_speed_hz
        self.spi.mode = 1
        self.spi.bits_per_word = 8
        self.spi.lsbfirst = False
        self.debug = debug
        
        print(f"SPI configured: mode={self.spi.mode}, speed={self.spi.max_speed_hz}Hz")
    
    def _send_receive_64(self, message: str):
        """Send message in 64-byte SPI frame and receive response"""
        # Add terminator
        full_message = message + '\r'
        
        # Convert string to byte list
        data_bytes = []
        for char in full_message:
            data_bytes.append(ord(char))
        
        # Pad to 64 bytes with zeros
        while len(data_bytes) < 64:
            data_bytes.append(0)
        
        # Ensure exactly 64 bytes
        data_bytes = data_bytes[:64]
        
        if self.debug:
            # Show what we're sending
            hex_str = ' '.join([f'{b:02X}' for b in data_bytes[:20]])
            ascii_str = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data_bytes[:30]])
            print(f"TX HEX: {hex_str}")
            print(f"TX STR: [{ascii_str}]")
        
        # Transfer
        rx_bytes = self.spi.xfer2(data_bytes)
        
        # CRITICAL: Wait for STM32 to process and prepare response
        time.sleep(0.01)  # 10ms delay
        
        # Send dummy transfer to receive the response
        dummy_bytes = [0x00] * 64
        rx_bytes = self.spi.xfer2(dummy_bytes)
        
        if self.debug:
            # Show what we received
            rx_hex = ' '.join([f'{b:02X}' for b in rx_bytes[:20]])
            rx_ascii = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in rx_bytes[:30]])
            print(f"RX HEX: {rx_hex}")
            print(f"RX STR: [{rx_ascii}]")
        
        # Parse response - filter out zeros
        rx_clean_bytes = [b for b in rx_bytes if b != 0]
        if rx_clean_bytes:
            try:
                response = bytes(rx_clean_bytes).decode('ascii', errors='ignore')
                return response.strip()
            except:
                return ""
        return ""
    
    def servo_set(self, pwm_value: int):
        """Send Servo PWM command to STM32 (1000-2000)"""
        cmd = f"CMD,SER,{pwm_value}"
        print(f"\nSending command: {cmd}")
        response = self._send_receive_64(cmd)
        return response
    
    def adc_read(self):
        """Query ADC value from STM32"""
        response = self._send_receive_64("QRY,ADC")
        return response
    
    def close(self):
        self.spi.close()

if __name__ == "__main__":
    client = SpiRpcClient(debug=True)
    
    # Servo PWM values
    servo_pwm_values = [1000, 1500, 2000, 1000]
    delay_s = 0.2  # 1 second delay
    
    try:
        print("\nStarting servo control loop...\n")
        while True:
            for pwm in servo_pwm_values:
                response = client.servo_set(pwm)
                print(f"Response: [{response}]")
                print("-" * 60)
                time.sleep(delay_s)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        client.close()
