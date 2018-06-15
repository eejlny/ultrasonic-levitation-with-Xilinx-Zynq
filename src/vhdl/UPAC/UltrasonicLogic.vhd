library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity UltrasonicLogic is
	generic (
    TRANSDUCER_NUM : integer := 64
    );
  Port (   
           Emmitter_clk : in STD_LOGIC; 
           Control : in STD_LOGIC_VECTOR(3 downto 0);
           Mask :in STD_LOGIC_VECTOR(TRANSDUCER_NUM -1 downto 0);
           Match : in STD_LOGIC_VECTOR ((8*TRANSDUCER_NUM)-1 downto 0);
           Phases : out STD_LOGIC_VECTOR (TRANSDUCER_NUM-1 downto 0);
           Sync : out std_logic
           );
end UltrasonicLogic;

architecture Behavioral of UltrasonicLogic is
   
signal Reset_Reg : std_logic := '1';
signal Sync_Reg1 : std_logic;
signal Sync_Reg2 : std_logic;
signal Phase_Reg1 : std_logic_vector(TRANSDUCER_NUM-1 downto 0);
signal Phase_Reg2 : std_logic_vector(TRANSDUCER_NUM-1 downto 0);

signal Match_Reg : std_logic_vector((8*TRANSDUCER_NUM)-1 downto 0);

signal Count_Reg : std_logic_vector(7 downto 0) := "00000000";
signal ZeroShift : std_logic_vector(255 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000011111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111";
component upcounter is
    Port ( CLK : in STD_LOGIC;
           Q : out STD_LOGIC_VECTOR (7 downto 0);
           RST : in STD_LOGIC);
end component;

component PhaseLogic is
    Port ( 
           CLK : in STD_LOGIC;
           Count : in STD_LOGIC_VECTOR (7 downto 0);
           Match : in STD_LOGIC_VECTOR (7 downto 0);
           Phase : out STD_LOGIC;
           ZeroShift : in std_logic_vector(255 downto 0)
           );
end component;



begin

process(Emmitter_clk, Control, Mask)
 variable res : integer := 0;
 variable options : std_logic_vector(2 downto 0);
begin
    if rising_edge(Emmitter_clk) then
        -- Check if the device is enabled
        if Control(0) = '1' then 
            options := Control (3 downto 1);
            case options is
                when "000" =>
                    Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
                    Sync <= '0';
                    Phase_Reg2 <= Phase_Reg1;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match_Reg;
                when "001" =>
                    Phases <= Phase_Reg2;
                    Sync <= Sync_Reg2;
                    Phase_Reg2 <= Phase_Reg1;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match_Reg;
                when "010" =>
                    Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
                    Sync <= '0';
                    Phase_Reg2 <= Phase_Reg1;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match;
                when "011" =>
                    Phases <= Phase_Reg2;
                    Sync <= Sync_Reg2;
                    Phase_Reg2 <= Phase_Reg1;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match;
                when "100" =>
                    Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
                    Sync <= '0';
                    Phase_Reg2 <= Phase_Reg1 AND Mask;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match_Reg;
                when "101" =>
                    Phases <= Phase_Reg2;
                    Sync <= Sync_Reg2;
                    Phase_Reg2 <= Phase_Reg1 AND Mask;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match_Reg;
                when "110" =>
                    Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
                    Sync <= '0';
                    Phase_Reg2 <= Phase_Reg1 AND Mask;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match;
                when "111" =>
                    Phases <= Phase_Reg2;
                    Sync <= Sync_Reg2;
                    Phase_Reg2 <= Phase_Reg1 AND Mask;
                    Sync_Reg2 <= Sync_Reg1;
                    Match_Reg <= Match;
                when others =>
            end case;
        else
            Sync <= '0';
            Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
        end if; 
        
        
        
        
--            when '0' =>
--                Reset_Reg <= '0';
--                Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
--                Sync <= '0';
--                Leds <= "01";
--            when '1' =>
--                Reset_Reg <= '1';
--                Phases <= Phase_Reg;
--                Sync <= Sync_Reg;
--                Leds <= "11";
--            when others =>
--                Reset_Reg <= '0';
--                Phases <= std_logic_vector(to_unsigned(res,TRANSDUCER_NUM));
--                Sync <= '0';
--                Leds <= "01";
--        end case;
    end if;
end process;

Counter:upcounter
port map(
    CLK => Emmitter_clk,
    Q => Count_Reg,
    RST => Reset_Reg
    );

GEN_COMP:
FOR n in TRANSDUCER_NUM-1 downto 0 generate
PhaseLogic0:PhaseLogic
    port map(
    CLK => Emmitter_clk,
    Count => Count_Reg,
    Match => Match_Reg(((n*8)+7) downto (n*8)),
    Phase => Phase_Reg1(n),
    ZeroShift => ZeroShift
    );
    end generate GEN_COMP;

Sync0:PhaseLogic
    port map(
    CLK => Emmitter_clk,
    Count => Count_Reg,
    Match => "00000000",
    Phase => Sync_Reg1,
    ZeroShift => ZeroShift
    );

end Behavioral;
