-- Engineer:     Vinicius Mylonas
-- Create Date:  20/01/2024 
-- Design Name:  Testbench FPGA PID controller 	
-- Module Name:  behavioral
-- Project Name: FPGA  PID controller	
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_pid is
end tb_pid;

architecture behavioral of tb_pid is
component pid
port (
    enable_in       : in  std_logic;    
    clk             : in  std_logic;
    setpoint_val_in : in  std_logic_vector(11 downto 0); -- valor de entrada
    output_val_out  : out std_logic_vector(11 downto 0)  -- valor de saída
);
end component;
     
    signal     setpoint_val_tb      : integer := 0;
    signal     setpoint_val_std_tb  : std_logic_vector(11 downto 0);
    signal     enable_tb            : std_logic :='0';
    signal     output_val_tb        : std_logic_vector(11 downto 0);
    signal     clk                  : std_logic:='0';
    constant   clk_period           : time := 10ns;
    
begin

    setpoint_val_std_tb <= std_logic_vector(to_unsigned(setpoint_val_tb,setpoint_val_std_tb'length));

    uut1:pid port map
    (    
        setpoint_val_in         => setpoint_val_std_tb,
        enable_in               => enable_tb,
        clk                     => clk,
        output_val_out          => output_val_tb
    );

    -- gera clock
    clk_process:process
    begin 
        wait for clk_period/2;
        clk <= not clk;
        wait for clk_period/2;
    end process;
    
    -- seleciona um setpoint desejado
    changed_set_point_value:process
    begin 
        wait for 150 ns;
        setpoint_val_tb <= 1000;
    end process;
    
    -- faz o reset inicial do sistema e então inicializa
    switch_process:process
    begin 
        enable_tb <= '0';
        wait for 100 ns;
        enable_tb <= '1';
        wait for 100 ms;
        enable_tb <= '0';
    end process;
end behavioral;