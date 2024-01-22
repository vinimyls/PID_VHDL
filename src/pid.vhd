-- Engineer:     Vinicius Mylonas
-- Create Date:  20/01/2024 
-- Design Name:  FPGA PID controller 	
-- Module Name:  behavioral
-- Project Name: FPGA  PID controller	
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pid is
    port (
        enable_in       : in  std_logic;    
        clk             : in  std_logic;
        setpoint_val_in : in  std_logic_vector(11 downto 0); -- valor de entrada
        output_val_out  : out std_logic_vector(11 downto 0)  -- valor de saída
    );
end pid;

architecture behavioral of pid is

    -- valores constantes kp, kd, ki, tempo
    constant con_kp          : integer := 1;
    constant con_kp_den      : integer := 2;
    constant con_kd          : integer := 1;
    constant con_kd_den      : integer := 100;
    constant con_ki          : integer := 1;
    constant con_ki_den      : integer := 10;
    constant tempo_divi      : integer := 1;

    -- sinais internos
    
    -- erros
    signal erro_atual        : integer;
    signal erro_diferenca    : integer;
    signal erro_soma         : integer;

    -- p i d 
    signal p, i, d           : integer;

    -- valores registrados
    signal realimentacao_reg : integer;
    signal erro_soma_reg     : integer;
    signal erro_reg          : integer;
    signal output_loaded     : integer;

    -- entrada / saida convertido para inteiro
    signal setpoint_val_int  : integer;
    signal output_val_int    : integer;
begin

    -- converte entrada / saida
    setpoint_val_int <= to_integer(unsigned(setpoint_val_in));
    output_val_out   <= std_logic_vector(to_unsigned(output_val_int, output_val_out'length));


    -- calcula erros
    erro_atual       <= setpoint_val_int - realimentacao_reg;
    erro_soma        <= erro_atual + erro_soma_reg;
    erro_diferenca   <= erro_atual - erro_reg;

    -- calcula pid
    p                <= (con_kp * erro_atual)/con_kp_den;
    i                <= (con_ki * erro_soma)/(tempo_divi * con_ki_den);
    d                <= ((con_kd * erro_diferenca) * tempo_divi)/con_kd_den;

    -- aplica pid
    output_loaded    <= (p + i + d);

    process (clk)
    begin
        if clk'event and clk = '1' then
        
            -- estado de reset
            if enable_in = '0' then
                realimentacao_reg <= 0;
                output_val_int    <= 0;
                erro_reg          <= 0;
                erro_soma_reg     <= 0;
            
            -- registra valores
            else
                realimentacao_reg <= output_loaded;
                output_val_int    <= output_loaded;
                erro_reg          <= erro_atual;
                erro_soma_reg     <= erro_soma;
            end if;
        end if;
    end process;
end behavioral;