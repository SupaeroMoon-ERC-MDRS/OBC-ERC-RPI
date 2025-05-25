from time import time_ns, sleep
from subprocess import run

class DummyMessage:
    def __init__(self):
        self.rpi_3v7_wl_sw_a = 0
        self.rpi_3v3_sys_a = 0
        self.rpi_1v8_sys_a = 0
        #self.ddr_vdd2_a = 0
        #self.ddr_vddq_a = 0
        self.rpi_1v1_sys_a = 0
        self.rpi_0v8_sw_a = 0
        self.vdd_core_a = 0
        #self.3v3_dac_a = 0
        #self.3v3_adc_a = 0
        #self.0v8_aon_a = 0
        #self.hdmi_a = 0
        self.rpi_3v7_wl_sw_v = 0
        self.rpi_3v3_sys_v = 0
        self.rpi_1v8_sys_v = 0
        #self.ddr_vdd2_v = 0
        #self.ddr_vddq_v = 0
        self.rpi_1v1_sys_v = 0
        self.rpi_0v8_sw_v = 0
        self.vdd_core_v = 0
        #self.3v3_dac_v = 0
        #self.3v3_adc_v = 0
        #self.0v8_aon_v = 0
        #self.hdmi_v = 0
        self.rpi_ext5v_v = 0
        #self.batt_v = 0
        self.rpi_temp = 0
        self.rpi_cpu = 0
        self.rpi_mem = 0


class RaspiStateChecker:
    __ELEC_COMMAND = ["vcgencmd", "pmic_read_adc"]
    __TEMP_COMMAND = ["vcgencmd", "measure_temp"]
    __TOP_COMMAND = ['top', '-b', '-n', '1', '-o', 'PID']

    __POLL_INTERVAL_MS = 10
    __POLL_INTERVAL = __POLL_INTERVAL_MS * 1000 * 1000

    __CPU_COUNT = 4

    def __init__(self):
        self.last_polled = time_ns()

    @staticmethod
    def __process_elec(raspi_state_msg, elec_stdio: bytes):
        lines = elec_stdio.split(b'\n')
        
        raspi_state_msg.rpi_3v7_wl_sw_a = float(lines[0].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_3v3_sys_a = float(lines[1].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_1v8_sys_a = float(lines[2].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_1v1_sys_a = float(lines[5].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_0v8_sw_a = float(lines[6].split(b'=')[-1][:-1])
        raspi_state_msg.vdd_core_a = float(lines[7].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_3v7_wl_sw_v = float(lines[12].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_3v3_sys_v = float(lines[13].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_1v8_sys_v = float(lines[14].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_1v1_sys_v = float(lines[17].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_0v8_sw_v = float(lines[18].split(b'=')[-1][:-1])
        raspi_state_msg.vdd_core_v = float(lines[19].split(b'=')[-1][:-1])
        raspi_state_msg.rpi_ext5v_v = float(lines[24].split(b'=')[-1][:-1])

    @staticmethod
    def __process_temp(raspi_state_msg, temp_stdio: bytes):
        raspi_state_msg.rpi_temp = float(temp_stdio[5:-3])

    @staticmethod
    def __process_top(raspi_state_msg, top_stdio: bytes):
        lines = top_stdio.split(b'\n')[7:]
        lines = lines[:min(len(lines), 10)]
        cpu, mem = 0, 0

        for l in lines:
            v = l.split(b' ')

            while(True):
                try:
                    v.remove(b'')
                except ValueError:
                    break
            if len(v) == 0:
                break

            cpu += float(v[8])
            mem += float(v[9])
        raspi_state_msg.rpi_cpu = cpu / RaspiStateChecker.__CPU_COUNT
        raspi_state_msg.rpi_mem = mem

    def poll(self, raspi_state_msg) -> bool:
        if time_ns() - self.last_polled < RaspiStateChecker.__POLL_INTERVAL:
            return False
        
        self.last_polled = time_ns()

        try:
            res = run(RaspiStateChecker.__ELEC_COMMAND, capture_output=True)
            if res.returncode:
                print(f"The command {RaspiStateChecker.__ELEC_COMMAND} failed with: {res.stderr}")
            res.check_returncode()
            RaspiStateChecker.__process_elec(raspi_state_msg, res.stdout)
        except Exception:
            print(f"The command {RaspiStateChecker.__ELEC_COMMAND} failed")
            
        try:
            res = run(RaspiStateChecker.__TEMP_COMMAND, capture_output=True)
            if res.returncode:
                print(f"The command {RaspiStateChecker.__TEMP_COMMAND} failed with: {res.stderr}")
            res.check_returncode()
            RaspiStateChecker.__process_temp(raspi_state_msg, res.stdout)
        except Exception:
            print(f"The command {RaspiStateChecker.__TEMP_COMMAND} failed")

        try:
            res = run(RaspiStateChecker.__TOP_COMMAND, capture_output=True)

            if res.returncode:
                print(f"The command {RaspiStateChecker.__TOP_COMMAND} failed with: {res.stderr}")
            res.check_returncode()
            RaspiStateChecker.__process_top(raspi_state_msg, res.stdout)
        except Exception as e:
            print(f"The command {RaspiStateChecker.__TOP_COMMAND} failed {e}")

        return True


if __name__ == '__main__':
    msg = DummyMessage()
    rsc = RaspiStateChecker()
    sleep(1)
    print(rsc.poll(msg))
    sleep(1)
    print(rsc.poll(msg))
