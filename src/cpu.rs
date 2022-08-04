// https://www.nesdev.org/wiki/CPU_registers
pub struct CPU {
    //A is byte-wide and along with the arithmetic logic unit (ALU),
    // supports using the status register for carrying, overflow detection, and so on.
    pub register_a: u8,
    // X and Y are byte-wide and used for several addressing modes.
    // They can be used as loop counters easily, using INC/DEC and branch instructions.
    // Not being the accumulator, they have limited addressing modes themselves when loading and saving.
    pub register_x: u8,
    pub register_y: u8,
    // P has 6 bits used by the ALU but is byte-wide. PHP, PLP, arithmetic, testing, and branch instructions can access this register.
    /// # Status Register (P) http://wiki.nesdev.com/w/index.php/Status_flags
    ///
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag 00000001
    ///  | |   | | | +----- Zero Flag  00000010
    ///  | |   | | +------- Interrupt Disable 00000100
    ///  | |   | +--------- Decimal Mode (not used on NES) 00001000
    ///  | |   +----------- Break Command 00010000
    ///  | +--------------- Overflow Flag 01000000
    ///  +----------------- Negative Flag 10000000
    ///
    pub status: u8,
    // S is byte-wide and can be accessed using interrupts, pulls, pushes, and transfers.
    pub stack_pointer: u8,
    // The 2-byte program counter PC supports 65536 direct (unbanked) memory locations,
    // however not all values are sent to the cartridge. It can be accessed either by allowing CPU's
    // internal fetch logic increment the address bus, an interrupt (NMI, Reset, IRQ/BRQ), and using the RTS/JMP/JSR/Branch instructions.
    pub program_counter: u16,
    // https://www.nesdev.org/wiki/CPU_memory_map
    // Address range	Size	Device
    // $0000-$07FF	$0800	2KB internal RAM
    // $0800-$0FFF	$0800	Mirrors of $0000-$07FF
    // $1000-$17FF	$0800
    // $1800-$1FFF	$0800
    // $2000-$2007	$0008	NES PPU registers
    // $2008-$3FFF	$1FF8	Mirrors of $2000-2007 (repeats every 8 bytes)
    // $4000-$4017	$0018	NES APU and I/O registers
    // $4018-$401F	$0008	APU and I/O functionality that is normally disabled. See CPU Test Mode.
    // $4020-$FFFF	$BFE0	Cartridge space: PRG ROM, PRG RAM, and mapper registers (See Note)
    // 64KB
    memory: [u8; 0xFFFF],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            status: 0,
            stack_pointer: 0,
            program_counter: 0,
            memory: [0; 0xFFFF],
        }
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&mut self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x8000);
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.status = 0;

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    fn update_zero_flag(&mut self, result: u8) {
        if result == 0 {
            //设置标志位为1
            self.status = self.status | 0b0000_0010;
        } else {
            //保留除标志位的其他位
            self.status = self.status & 0b1111_1101;
        }
    }

    fn update_negative_flag(&mut self, result: u8) {
        if result & 0b1000_0000 != 0 {
            self.status = self.status | 0b1000_0000;
        } else {
            self.status = self.status & 0b0111_1111;
        }
    }

    fn tax(&mut self) {
        //Transfer A to X
        self.register_x = self.register_a;

        //Affects Flags: N Z
        //根据register_a改变status标志
        //update Zero Flag - if that value is zero, this flag will be set
        self.update_zero_flag(self.register_x);

        //update Negative Flag - 根据register_a标志位设置status标志
        self.update_negative_flag(self.register_x);
    }

    fn lda(&mut self, value: u8) {
        self.register_a = value;

        // http://www.6502.org/tutorials/6502opcodes.html#LDA
        //LDA Affects Flags: N Z

        //根据register_a改变status标志
        //update Zero Flag - if that value is zero, this flag will be set
        self.update_zero_flag(self.register_a);

        //update Negative Flag - 根据register_a标志位设置status标志
        self.update_negative_flag(self.register_a);
    }

    fn inx(&mut self) {
        //INcrement X
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_flag(self.register_x);
        self.update_negative_flag(self.register_x);
    }

    pub fn run(&mut self) {
        loop {
            let opscode = self.mem_read(self.program_counter);
            self.program_counter += 1;
            // http://www.6502.org/tutorials/6502opcodes.html
            match opscode {
                0xE8 => {
                    self.inx();
                }
                0xAA => {
                    self.tax();
                }
                0xA9 => {
                    //Mode: Immediate
                    //读取opscode的下一条参数
                    let param = self.mem_read(self.program_counter);
                    self.program_counter += 1;
                    self.lda(param);
                }
                0x00 => {
                    return;
                }
                _ => todo!()
            }
        }
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert_eq!(cpu.status & 0b0000_0010, 0b10);
    }

    //0x05 -> 0000 0101
    //0x87 -> 1000 0111
    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x87, 0x00]);
        assert_eq!(cpu.register_a, 0x87);
        assert_eq!(cpu.status & 0b0000_0010, 0b00);
        //0x87 - zero flag 后应该为 1000 0101
        //negative flag 后应该为 1000 0101
        assert_eq!(cpu.status & 0b1000_0000, 0b1000_0000);
    }


    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xaa, 0x00]);
        cpu.register_a = 10;
        cpu.program_counter = cpu.mem_read_u16(0xFFFC);
        cpu.run();

        assert_eq!(cpu.register_x, 10)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xe8, 0xe8, 0x00]);
        cpu.register_x = 0xff;
        cpu.program_counter = cpu.mem_read_u16(0xFFFC);
        cpu.run();

        assert_eq!(cpu.register_x, 1)
    }
}