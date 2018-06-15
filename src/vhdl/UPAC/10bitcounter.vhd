----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 01.02.2017 17:18:39
-- Design Name: 
-- Module Name: counter - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity upcounter is
    Port ( CLK : in STD_LOGIC;
           Q : out STD_LOGIC_VECTOR (7 downto 0);
           RST : in STD_LOGIC);
end upcounter;

architecture Behavioral of upcounter is
signal value: STD_LOGIC_VECTOR (7 downto 0) := "00000000";
begin

count:process(CLK,RST)
begin
    if rising_edge(clk) then
        if RST='0' then
            value <= "00000000";
        else
            
                value <= value +1;
                Q<=value;
        end if;
        
    end if;
end process;



end Behavioral;
