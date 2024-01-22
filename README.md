# FPGA PID controller

![Header](/pictures/header.png "header")

O controle proporcional-integral-derivativo (PID) consolidou-se como uma ferramenta fundamental no domínio da engenharia de controle. Sua aplicação abrange uma ampla gama de setores, desde sistemas industriais complexos até dispositivos eletrônicos cotidianos. A popularidade do controle PID reside na sua capacidade única de proporcionar estabilidade e precisão em sistemas dinâmicos, garantindo um desempenho otimizado em diferentes contextos.
Este artigo tem como objetivo explicar a implementação de um controlador PID simples em FPGA. Para uma introdução mais aprofundada sobre controladores PID, recomenda-se a leitura do artigo [Controladores PID com a FRDM KL25Z](https://embarcados.com.br/controladores-pid-com-a-frdm-kl25z/).

## Aproveitando o paralelismo

Por que escolher um FPGA em vez de um microcontrolador para implementar um PID simples? Embora microcontroladores possam ser suficientes para projetos simples, a escolha de um FPGA é particularmente relevante em situações críticas.

Os microcontroladores podem apresentar latências mais altas, especialmente em execuções sequenciais de código. Isso pode impactar a capacidade do sistema de manter uma resposta rápida a perturbações ou mudanças nos setpoints. Por outro lado, os FPGAs oferecem baixa latência e alta velocidade de resposta devido à natureza paralela de seu processamento. Essa característica é crucial em sistemas de controle em tempo real, onde a rápida adaptação às mudanças nas condições é essencial.

Outra vantagem digna de nota é a elegância da descrição em VHDL, que é simples de entender e desenvolvida com pouca complexidade, facilitando a portabilidade desses conhecimentos para outros projetos por estudantes e entusiastas.

## Diagrama de implementação

O port "setpoint_val_in" pode ser interpretado como um valor recebido de um conversor analógico-digital (ADC), enquanto o port "output_val_out" pode ser interpretado como uma saída para um conversor digital-analógico (DAC). O port "enable" inicializa e faz o reset dos valores armazenados no circuito. A seguir, os detalhes dos ports e valores constantes:

```vhdl
    -- Ports:
    enable_in       : in  std_logic;    
    clk             : in  std_logic;
    setpoint_val_in : in  std_logic_vector(11 downto 0); -- valor de entrada
    output_val_out  : out std_logic_vector(11 downto 0)  -- valor de saída
```
Os valores de kp, kd, ki, os denominadores kp_den, kd_den, ki_den, e a constante de tempo estão armazenados como constantes, conforme mostrado abaixo:
```vhdl
    -- valores constantes kp, kd, ki, tempo
    constant con_kp          : integer := 1;
    constant con_kp_den      : integer := 2;
    constant con_kd          : integer := 1;
    constant con_kd_den      : integer := 100;
    constant con_ki          : integer := 1;
    constant con_ki_den      : integer := 10;
    constant tempo_divi      : integer := 1;
```
Os valores para a próximo ciclo devem ser armazenados em registradores:
```vhdl
    -- valores registrados
    signal realimentacao_reg : integer;
    signal erro_soma_reg     : integer;
    signal erro_reg          : integer;
    signal output_loaded     : integer;
```
Os erros e os valores de P, I e D são calculados a partir do momento em que o enable do sistema é ativado:
 ```vhdl
    -- calculados os erros:
    erro_atual       <= setpoint_val_in - realimentacao_reg;
    erro_soma        <= erro_atual + erro_soma_reg;
    erro_diferenca  <= erro_atual - erro_reg;
```
Com os valores definidos, o valor de PID pode ser recalculado usando as seguintes fórmulas:
```vhdl
    -- calculando P, I e D 
    p                <= (con_kp * erro_atual)/con_kp_den;
    i                <= (con_ki * erro_soma)/(divider_for_time * con_ki_den);
    d                <= ((con_kd * erro_diferenca) * divider_for_time)/con_kd_den;
```
E, finalmente, o valor de saída e realimentação, que serão atualizados na próxima borda de subida do clock, podem ser calculados:
```vhdl
    -- aplica pid
    output_loaded    <= (p + i + d);
```
Os valores são armazenados para os cálculos no próximo ciclo:
```vhdl
    -- Armazenando os valores para o próximo ciclo
    realimentacao_reg <= output_loaded;
    output_val_out    <= output_loaded;
    erro_reg          <= erro_atual;
    erro_soma_reg     <= erro_soma;
```

O diagrama que ilustra essa implementação pode ser visto a seguir:
![PID diagrama](/pictures/PID_drawio.png "PID diagrama")

## Implementação do Controle PID em VHDL para FPGAs

Integrando todas as partes anteriores e adicionando as lógicas de registradores, temos o seguinte arquivo VHDL:

```vhdl
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

    -- entrada / saída convertido para inteiro
    signal setpoint_val_int  : integer;
    signal output_val_int    : integer;
begin

    -- converte entrada / saída
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
```

## Testbench
O Testbench (TB) deste projeto não apresenta grandes diferenças em relação a qualquer outro. Ao fornecer um valor, verificamos o estímulo e o resultado. Um pequeno detalhe que gostaria de acrescentar é que o "setpoint_val_tb" é um valor inteiro, que é convertido para std_logic_vector(11 downto 0), facilitando o teste com múltiplos estímulos.

```vhdl
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
```

forma de onda:
![Forma de onda](/pictures/wave.png "Forma de onda")

## Conclusão

Este artigo destaca a relevância e a versatilidade da implementação do controle PID em VHDL para FPGAs. Ao integrar princípios sólidos de controle com a flexibilidade da VHDL, esta abordagem não só atende às demandas de sistemas dinâmicos complexos, mas também proporciona uma base sólida para futuros desenvolvimentos na área de controle embarcado. A convergência do PID e VHDL emerge como uma solução poderosa, proporcionando um controle preciso e eficiente em ambientes que exigem desempenho otimizado.

Este projeto está disponível em meu GitHub e é livre para ser replicado. Este exemplo foca em cálculos com números inteiros; para utilizar valores de ponto flutuante, algumas alterações nos tipos são necessárias.

### Referências 

[pid Controller VHDL : 10 Steps - Instructables](https://www.instructables.com/PID-Controller-VHDL/)

[PID Deep Charkraborty](https://github.com/deepc94/pid-fpga-vhdl/blob/master/pid.vhd)