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

// https://www.jianshu.com/p/579682bb2f9c
// https://www.jianshu.com/p/b909689db484
#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    // 直接在指令中指定一个8位常量
    Immediate,
    // 使用零页寻址模式的指令只有一个8位地址操作数。这限制了它只能寻址前256字节的内存
    ZeroPage,
    // 使用零页，X寄存器寻址时，指令将8位零页地址和X寄存器中加载的值进行累加运算，得到的数据就是直接寻址的地址
    ZeroPage_X,
    // 使用零页，Y寄存器寻址时，指令将8位零页地址和Y寄存器中加载的值进行累加运算，得到的数据就是直接寻址的地址
    ZeroPage_Y,
    // 使用绝对寻址的指令需要包含一个完整的16位地址来识别目标位置
    Absolute,
    // 使用X寄存器索引进行绝对寻址访问地址的方法是通过s指令获取16位地址与X寄存器的中内容累加来计算得到的
    Absolute_X,
    // 将Y寄存器内存储的数据与一个16位地址相加，从而得到需要访问的地址
    Absolute_Y,
    // http://www.emulator101.com/6502-addressing-modes.html
    // 从指令中获取表的地址，并向其添加X寄存器(零页循环)，以给出目标地址的最低有效字节的位置
    // 索引间接寻址（Indexed Indirect）
    Indirect_X,
    // 在指令中包含16位地址的最低有效字节的零页位置。Y寄存器被动态添加到这个值中，以生成操作的实际目标地址。
    // 间接变址寻址（Indirect Indexed）
    Indirect_Y,
    NoneAddressing,
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

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => {
                self.program_counter
            }
            AddressingMode::ZeroPage => {
                self.mem_read(self.program_counter) as u16
            }
            AddressingMode::Absolute => {
                self.mem_read_u16(self.program_counter)
            }
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }
            AddressingMode::Absolute_X => {
                let pos = self.mem_read_u16(self.program_counter);
                let addr = pos.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let pos = self.mem_read_u16(self.program_counter);
                let addr = pos.wrapping_add(self.register_y as u16);
                addr
            }
            // https://zhuanlan.zhihu.com/p/458206025
            // 先变址后间接
            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                //变址
                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                //间接
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            // 先间接后变址
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                //间接
                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                //变址
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&self, pos: u16) -> u16 {
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
        //根据register_x改变status标志
        //update Zero Flag - if that value is zero, this flag will be set
        self.update_zero_flag(self.register_x);

        //update Negative Flag - 根据register_x标志位设置status标志
        self.update_negative_flag(self.register_x);
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
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
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            // http://www.6502.org/tutorials/6502opcodes.html
            match code {
                0xE8 => {
                    self.inx();
                }
                0xAA => {
                    self.tax();
                }
                0xA9 => {
                    self.lda(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xA5 => {
                    self.lda(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0xB5 => {
                    self.lda(&AddressingMode::ZeroPage_X);
                    self.program_counter += 1;
                }
                0xAD => {
                    self.lda(&AddressingMode::Absolute);
                    self.program_counter += 1;
                }
                0xBD => {
                    self.lda(&AddressingMode::Absolute_X);
                    self.program_counter += 1;
                }
                0xB9 => {
                    self.lda(&AddressingMode::Absolute_Y);
                    self.program_counter += 1;
                }
                0xA1 => {
                    self.lda(&AddressingMode::Indirect_X);
                    self.program_counter += 1;
                }
                0xB1 => {
                    self.lda(&AddressingMode::Indirect_Y);
                    self.program_counter += 1;
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

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }
}