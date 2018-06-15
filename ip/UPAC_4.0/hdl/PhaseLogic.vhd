library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
library UNISIM;
use UNISIM.VComponents.all;

entity PhaseLogic is
    Port ( 
           CLK : in STD_LOGIC;
           Count : in STD_LOGIC_VECTOR (7 downto 0);
           Match : in STD_LOGIC_VECTOR (7 downto 0);
           Phase : out STD_LOGIC;
           ZeroShift : in std_logic_vector(255 downto 0)
           );
end PhaseLogic;

architecture Behavioral of PhaseLogic is

begin

checker: process(CLK,Count,Match)
variable CurrentShift : std_logic_vector(255 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
variable temp : std_logic := '0';
variable LookupPos : unsigned(7 downto 0) := "00000000";
begin
-- New Method
if rising_edge(clk) then
    LookupPos := unsigned(Count) - unsigned(Match);
    Phase <= ZeroShift(to_integer(unsigned('0' & LookupPos)));
end if;

-- Shifting method
--if rising_edge(CLK) then
--    CurrentShift := std_logic_vector(unsigned(ZeroShift) rol to_integer(unsigned('0' & Match)));         
--                 if CurrentShift(to_integer(unsigned('0' & Count))) = '1' then
--                    Phase <= '1';
--                 else
--                    Phase <= '0';
--                 end if;
             
      
--end if;

end process; 
  
end Behavioral;
