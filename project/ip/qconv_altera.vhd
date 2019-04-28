-----------------------------------------------------------------------------------
--!     @file    qconv_params.vhd
--!     @brief   Quantized Convolution Parameters Package
--!     @version 0.1.0
--!     @date    2019/3/20
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
-----------------------------------------------------------------------------------
--! @brief Quantized Convolution のパラメータを定義しているパッケージ
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
package QCONV_PARAMS is

    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    type      QCONV_PARAMS_TYPE     is record
                  NUM_PE                   :  integer;
                  NBITS_PER_WORD           :  integer;
                  NBITS_IN_DATA            :  integer;
                  NBITS_K_DATA             :  integer;
                  NBITS_OUT_DATA           :  integer;
                  NUM_THRESHOLDS           :  integer;
                  MAX_IN_W                 :  integer;
                  MAX_IN_H                 :  integer;
                  MAX_IN_C                 :  integer;
                  MAX_IN_C_BY_WORD         :  integer;
                  MAX_OUT_C                :  integer;
                  MAX_OUT_W                :  integer;
                  MAX_OUT_H                :  integer;
                  MAX_K_W                  :  integer;
                  MAX_K_H                  :  integer;
                  MAX_PAD_SIZE             :  integer;
                  IN_C_BITS                :  integer;
                  IN_C_BY_WORD_BITS        :  integer;
                  IN_W_BITS                :  integer;
                  IN_H_BITS                :  integer;
                  OUT_C_BITS               :  integer;
                  OUT_W_BITS               :  integer;
                  OUT_H_BITS               :  integer;
                  K_W_BITS                 :  integer;
                  K_H_BITS                 :  integer;
                  PAD_SIZE_BITS            :  integer;
    end record;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  NEW_QCONV_PARAMS(
                  NUM_PE                   :  integer;
                  NBITS_PER_WORD           :  integer;
                  NBITS_IN_DATA            :  integer;
                  NBITS_K_DATA             :  integer;
                  NBITS_OUT_DATA           :  integer;
                  NUM_THRESHOLDS           :  integer;
                  MAX_K_W                  :  integer;
                  MAX_K_H                  :  integer;
                  MAX_PAD_SIZE             :  integer;
                  MAX_IN_C                 :  integer;
                  MAX_OUT_C                :  integer;
                  MAX_OUT_W                :  integer;
                  MAX_OUT_H                :  integer
    )             return                      QCONV_PARAMS_TYPE;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    constant  QCONV_COMMON_PARAMS          :  QCONV_PARAMS_TYPE := NEW_QCONV_PARAMS(
                                                  NUM_PE         => 16,    
                                                  NBITS_PER_WORD => 32,    -- 1 word     = 32bit
                                                  NBITS_IN_DATA  => 2,     -- 1 in_data  =  2bit
                                                  NBITS_K_DATA   => 1,     -- 1 k_data   =  1bit
                                                  NBITS_OUT_DATA => 16,    -- 1 out_data = 16bit
                                                  NUM_THRESHOLDS => 4,     --
                                                  MAX_K_W        => 3,     -- 
                                                  MAX_K_H        => 3,     -- 
                                                  MAX_PAD_SIZE   => 8,     -- 
                                                  MAX_IN_C       => 1024,  -- 
                                                  MAX_OUT_C      => 1024,  -- 
                                                  MAX_OUT_W      => 1024,  --
                                                  MAX_OUT_H      => 1024   -- 
                                              );
end     QCONV_PARAMS;
-----------------------------------------------------------------------------------
--! @brief Quantized Convolution のパラメータを定義しているパッケージ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
package body QCONV_PARAMS is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  max_to_bits(MAX:integer) return integer is
        variable bits : integer;
    begin
        bits := 1;
        while (2**bits <= MAX) loop
            bits := bits + 1;
        end loop;
        return bits;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  NEW_QCONV_PARAMS(
                  NUM_PE                   :  integer;
                  NBITS_PER_WORD           :  integer;
                  NBITS_IN_DATA            :  integer;
                  NBITS_K_DATA             :  integer;
                  NBITS_OUT_DATA           :  integer;
                  NUM_THRESHOLDS           :  integer;
                  MAX_K_W                  :  integer;
                  MAX_K_H                  :  integer;
                  MAX_PAD_SIZE             :  integer;
                  MAX_IN_C                 :  integer;
                  MAX_OUT_C                :  integer;
                  MAX_OUT_W                :  integer;
                  MAX_OUT_H                :  integer
    )             return                      QCONV_PARAMS_TYPE
    is
         variable params                   :  QCONV_PARAMS_TYPE;
    begin
        params.NUM_PE            := NUM_PE;
        params.NBITS_PER_WORD    := NBITS_PER_WORD;
        params.NBITS_IN_DATA     := NBITS_IN_DATA;
        params.NBITS_K_DATA      := NBITS_K_DATA;
        params.NBITS_OUT_DATA    := NBITS_OUT_DATA;
        params.NUM_THRESHOLDS    := NUM_THRESHOLDS;
        params.MAX_IN_W          := MAX_OUT_W + 2;
        params.MAX_IN_H          := MAX_OUT_H + 2;
        params.MAX_IN_C          := MAX_IN_C;
        params.MAX_IN_C_BY_WORD  := MAX_IN_C / params.NBITS_PER_WORD;
        params.MAX_OUT_C         := MAX_OUT_C;
        params.MAX_OUT_W         := MAX_OUT_W;
        params.MAX_OUT_H         := MAX_OUT_H;
        params.MAX_K_W           := MAX_K_W;
        params.MAX_K_H           := MAX_K_H;
        params.MAX_PAD_SIZE      := MAX_PAD_SIZE;
        params.IN_C_BITS         := max_to_bits(params.MAX_IN_C);
        params.IN_C_BY_WORD_BITS := max_to_bits(params.MAX_IN_C_BY_WORD);
        params.IN_W_BITS         := max_to_bits(params.MAX_IN_W);
        params.IN_H_BITS         := max_to_bits(params.MAX_IN_H);
        params.OUT_C_BITS        := max_to_bits(params.MAX_OUT_C);
        params.OUT_W_BITS        := max_to_bits(params.MAX_OUT_W);
        params.OUT_H_BITS        := max_to_bits(params.MAX_OUT_H);
        params.K_W_BITS          := max_to_bits(params.MAX_K_W);
        params.K_H_BITS          := max_to_bits(params.MAX_K_H);
        params.PAD_SIZE_BITS     := max_to_bits(params.MAX_PAD_SIZE);
        return params;
    end function;
end QCONV_PARAMS;
-----------------------------------------------------------------------------------
--!     @file    qconv_apply_thresholds.vhd
--!     @brief   Quantized Convolution Apply Thresholds Module
--!     @version 0.1.0
--!     @date    2019/3/20
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief Quantized Convolution Apply Thresholds Module
-----------------------------------------------------------------------------------
entity  QCONV_APPLY_THRESHOLDS is
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        I_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE IMAGE DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          --! * 次の条件を満していなければならない.
                          --!     I_PARAM.SHAPE = O_PARAM.SHAPE
                          --!     I_PARAM.SHAPE = K_PARAM.SHAPE
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        T_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE THRESHOLD DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          --! * 次の条件を満していなければならない.
                          --!     T_PARAM.SHAPE = I_PARAM.SHAPE
                          --!     T_PARAM.SHAPE = O_PARAM.SHAPE
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        O_PARAM         : --! @brief OUTPUT CONVOLUTION PIPELINE DATA PARAMETER :
                          --! パイプラインデータ出力ポートのパラメータを指定する.
                          --! * 次の条件を満していなければならない.
                          --!     O_PARAM.SHAPE = I_PARAM.SHAPE
                          --!     O_PARAM.SHAPE = T_PARAM.SHAPE
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        QUEUE_SIZE      : --! パイプラインレジスタの深さを指定する.
                          --! * QUEUE_SIZE=0 の場合は出力にキューが挿入されずダイレ
                          --!   クトに出力される.
                          integer := 2
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(I_PARAM.DATA.SIZE-1 downto 0);
        I_VALID         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * I_DATAが有効であることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        I_READY         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
        T_DATA          : --! @brief INPUT CONVOLUTION PIPELINE THRESHOLD DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(T_PARAM.DATA.SIZE-1 downto 0);
        T_VALID         : --! @brief INPUT CONVOLUTION PIPELINE THRESHOLD DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * T_DATAが有効であることを示す.
                          --! * T_VALID='1'and T_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        T_READY         : --! @brief INPUT CONVOLUTION PIPELINE THRESHOLD DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * T_VALID='1'and T_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 出力パイプラインデータ有効信号.
                          --! * O_DATA が有効であることを示す.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          out std_logic;
        O_READY         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 出力パイプラインデータレディ信号.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          in  std_logic
    );
end QCONV_APPLY_THRESHOLDS;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.COMPONENTS.PIPELINE_REGISTER;
architecture RTL of QCONV_APPLY_THRESHOLDS is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    subtype   I_ELEM_TYPE     is std_logic_vector(I_PARAM.ELEM_BITS-1 downto 0);
    type      I_ELEM_VECTOR   is array(0 to I_PARAM.SHAPE.Y.SIZE-1,
                                       0 to I_PARAM.SHAPE.X.SIZE-1,
                                       0 to I_PARAM.SHAPE.D.SIZE-1,
                                       0 to I_PARAM.SHAPE.C.SIZE-1) of I_ELEM_TYPE;
    signal    i_element       :  I_ELEM_VECTOR;
    signal    i_d_atrb        :  IMAGE_STREAM_ATRB_VECTOR(0 to I_PARAM.SHAPE.D.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    subtype   T_ELEM_TYPE     is std_logic_vector(T_PARAM.ELEM_BITS-1 downto 0);
    type      T_ELEM_VECTOR   is array(0 to T_PARAM.SHAPE.Y.SIZE-1,
                                       0 to T_PARAM.SHAPE.X.SIZE-1,
                                       0 to T_PARAM.SHAPE.D.SIZE-1,
                                       0 to T_PARAM.SHAPE.C.SIZE-1) of T_ELEM_TYPE;
    signal    t_element       :  T_ELEM_VECTOR;
    signal    t_d_atrb        :  IMAGE_STREAM_ATRB_VECTOR(0 to T_PARAM.SHAPE.D.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    subtype   O_ELEM_TYPE     is std_logic_vector(O_PARAM.ELEM_BITS-1 downto 0);
    type      O_ELEM_VECTOR   is array(0 to O_PARAM.SHAPE.Y.SIZE-1,
                                       0 to O_PARAM.SHAPE.X.SIZE-1,
                                       0 to O_PARAM.SHAPE.D.SIZE-1,
                                       0 to O_PARAM.SHAPE.C.SIZE-1) of O_ELEM_TYPE;
    signal    o_element       :  O_ELEM_VECTOR;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    q_data          :  std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
    signal    q_valid         :  std_logic;
    signal    q_ready         :  std_logic;
begin
    -------------------------------------------------------------------------------
    -- i_element : 入力パイプラインイメージデータを要素ごとの配列に変換
    -- i_d_valid : 入力パイプラインイメージデータのチャネル有効信号
    -------------------------------------------------------------------------------
    process (I_DATA) begin
        for y in 0 to I_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to I_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to I_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to I_PARAM.SHAPE.C.SIZE-1 loop
            i_element(y,x,d,c) <= GET_ELEMENT_FROM_IMAGE_STREAM_DATA(I_PARAM, c, d, x, y, I_DATA);
        end loop;
        end loop;
        end loop;
        end loop;
        i_d_atrb <= GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(I_PARAM, I_DATA);
    end process;
    -------------------------------------------------------------------------------
    -- t_element : 入力パイプライン THRESHOLD データを要素ごとの配列に変換
    -- t_d_valid : 入力パイプライン THRESHOLD データのチャネル有効信号
    -------------------------------------------------------------------------------
    process (T_DATA) begin
        for y in 0 to T_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to T_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to T_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to T_PARAM.SHAPE.C.SIZE-1 loop
            t_element(y,x,d,c) <= GET_ELEMENT_FROM_IMAGE_STREAM_DATA(T_PARAM, c, d, x, y, T_DATA);
        end loop;
        end loop;
        end loop;
        end loop;
        t_d_atrb <= GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(T_PARAM, T_DATA);
    end process;
    -------------------------------------------------------------------------------
    -- o_element : 演算結果
    -------------------------------------------------------------------------------
    process(i_element, t_element)
        variable i_data  :           signed(I_PARAM.ELEM_BITS  -1 downto 0);
        variable t_data  : std_logic_vector(T_PARAM.ELEM_BITS  -1 downto 0);
        variable t0      :           signed(T_PARAM.ELEM_BITS/4-1 downto 0);
        variable t1      :           signed(T_PARAM.ELEM_BITS/4-1 downto 0);
        variable t2      :           signed(T_PARAM.ELEM_BITS/4-1 downto 0);
        variable s_flag  :           signed(T_PARAM.ELEM_BITS/4-1 downto 0);
        variable u_flag  :         unsigned(T_PARAM.ELEM_BITS/4-1 downto 0);
        variable o_data  :         unsigned(O_PARAM.ELEM_BITS  -1 downto 0);
    begin
        for y in 0 to O_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to O_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to O_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to O_PARAM.SHAPE.C.SIZE-1 loop
            i_data := to_01(signed(i_element(y,x,d,c)));
            t_data := t_element(y,x,d,c);
            t0     := to_01(  signed(t_data((0+1)*QCONV_PARAM.NBITS_OUT_DATA-1 downto 0*QCONV_PARAM.NBITS_OUT_DATA)));
            t1     := to_01(  signed(t_data((1+1)*QCONV_PARAM.NBITS_OUT_DATA-1 downto 1*QCONV_PARAM.NBITS_OUT_DATA)));
            t2     := to_01(  signed(t_data((2+1)*QCONV_PARAM.NBITS_OUT_DATA-1 downto 2*QCONV_PARAM.NBITS_OUT_DATA)));
            s_flag := to_01(  signed(t_data((3+1)*QCONV_PARAM.NBITS_OUT_DATA-1 downto 3*QCONV_PARAM.NBITS_OUT_DATA)));
            u_flag := to_01(unsigned(t_data((3+1)*QCONV_PARAM.NBITS_OUT_DATA-1 downto 3*QCONV_PARAM.NBITS_OUT_DATA)));
            if    (s_flag = 1) then
                if    (i_data < t0) then
                    o_data := to_unsigned(0, O_PARAM.ELEM_BITS);
                elsif (i_data < t1) then
                    o_data := to_unsigned(1, O_PARAM.ELEM_BITS);
                elsif (i_data < t2) then
                    o_data := to_unsigned(2, O_PARAM.ELEM_BITS);
                else
                    o_data := to_unsigned(3, O_PARAM.ELEM_BITS);
                end if;
            elsif (s_flag = -1) then
                if    (i_data > t2) then
                    o_data := to_unsigned(0, O_PARAM.ELEM_BITS);
                elsif (i_data > t1) then
                    o_data := to_unsigned(1, O_PARAM.ELEM_BITS);
                elsif (i_data > t0) then
                    o_data := to_unsigned(2, O_PARAM.ELEM_BITS);
                else
                    o_data := to_unsigned(3, O_PARAM.ELEM_BITS);
                end if;
            else
                o_data := resize((u_flag - 2), O_PARAM.ELEM_BITS);
            end if;
            o_element(y,x,d,c) <= std_logic_vector(o_data);
        end loop;
        end loop;
        end loop;
        end loop;
    end process;
    -------------------------------------------------------------------------------
    -- q_data    : パイプラインレジスタに入力するデータ
    -------------------------------------------------------------------------------
    process (o_element, I_DATA)
        variable data :  std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
    begin
        for y in 0 to O_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to O_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to O_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to O_PARAM.SHAPE.C.SIZE-1 loop
            SET_ELEMENT_TO_IMAGE_STREAM_DATA(O_PARAM, c, d, x, y, o_element(y,x,d,c), data);
        end loop;        
        end loop;        
        end loop;
        end loop;
        if (O_PARAM.DATA.ATRB_FIELD.SIZE > 0) then
            data(O_PARAM.DATA.ATRB_FIELD.HI downto O_PARAM.DATA.ATRB_FIELD.LO) := I_DATA(I_PARAM.DATA.ATRB_FIELD.HI downto I_PARAM.DATA.ATRB_FIELD.LO);
        end if;
        if (O_PARAM.DATA.INFO_FIELD.SIZE > 0) then
            data(O_PARAM.DATA.INFO_FIELD.HI downto O_PARAM.DATA.INFO_FIELD.LO) := I_DATA(I_PARAM.DATA.INFO_FIELD.HI downto I_PARAM.DATA.INFO_FIELD.LO);
        end if;
        q_data <= data;
    end process;
    -------------------------------------------------------------------------------
    -- q_valid   : 
    -------------------------------------------------------------------------------
    q_valid <= '1' when (I_VALID = '1' and T_VALID = '1') else '0';
    I_READY <= '1' when (q_valid = '1' and q_ready = '1') else '0';
    T_READY <= '1' when (q_valid = '1' and q_ready = '1') else '0';
    -------------------------------------------------------------------------------
    -- パイプラインレジスタ
    -------------------------------------------------------------------------------
    QUEUE: PIPELINE_REGISTER                   -- 
        generic map (                          -- 
            QUEUE_SIZE  => QUEUE_SIZE        , --
            WORD_BITS   => O_PARAM.DATA.SIZE   -- 
        )                                      -- 
        port map (                             -- 
            CLK         => CLK               , -- In  :
            RST         => RST               , -- In  :
            CLR         => CLR               , -- In  :
            I_WORD      => q_data            , -- In  :
            I_VAL       => q_valid           , -- In  :
            I_RDY       => q_ready           , -- Out :
            Q_WORD      => O_DATA            , -- Out :
            Q_VAL       => O_VALID           , -- Out :
            Q_RDY       => O_READY           , -- In  :
            BUSY        => open                -- Out :
        );                                     -- 
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_components.vhd                                            --
--!     @brief   Quantized Convolution Component Library                         --
--!     @version 0.1.0                                                           --
--!     @date    2019/04/27                                                      --
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>                     --
-----------------------------------------------------------------------------------
-----------------------------------------------------------------------------------
--                                                                               --
--      Copyright (C) 2019 Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>           --
--      All rights reserved.                                                     --
--                                                                               --
--      Redistribution and use in source and binary forms, with or without       --
--      modification, are permitted provided that the following conditions       --
--      are met:                                                                 --
--                                                                               --
--        1. Redistributions of source code must retain the above copyright      --
--           notice, this list of conditions and the following disclaimer.       --
--                                                                               --
--        2. Redistributions in binary form must reproduce the above copyright   --
--           notice, this list of conditions and the following disclaimer in     --
--           the documentation and/or other materials provided with the          --
--           distribution.                                                       --
--                                                                               --
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS      --
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT        --
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    --
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    --
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    --
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         --
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    --
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    --
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      --
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    --
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     --
--                                                                               --
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
-----------------------------------------------------------------------------------
--! @brief Quantized Convolution Component Library                               --
-----------------------------------------------------------------------------------
package QCONV_COMPONENTS is
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_AXI_CORE                                                  --
-----------------------------------------------------------------------------------
component QCONV_STRIP_AXI_CORE
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        ID                  : --! @brief QCONV ID STRING :
                              string(1 to 8) := "QCONV-S1";
        IN_BUF_SIZE         : --! @brief IN DATA BUFFER SIZE :
                              --! 入力バッファの容量を指定する.
                              --! * ここで指定する単位は1ワード単位.
                              --! * 1ワードは QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD
                              --!   = 64 bit.
                              --! * 入力バッファの容量は 入力チャネル × イメージの幅.
                              integer := 512*4*1;  -- 512word × BANK_SIZE × IN_C_UNROLL 
        K_BUF_SIZE          : --! @brief K DATA BUFFER SIZE :
                              --! カーネル係数バッファの容量を指定する.
                              --! * ここで指定する単位は1ワード単位.
                              --! * 1ワードは 3 * 3 * QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                              --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                              integer := 512*3*3*8*1;  -- 512word × 3 × 3 × OUT_C_UNROLL × IN_C_UNROLL
        TH_BUF_SIZE         : --! @brief THRESHOLDS DATA BUFFER SIZE :
                              --! THRESHOLDS バッファの容量を指定する.
                              --! * ここで指定する単位は1ワード単位.
                              --! * 1ワードは QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS
                              --! * = 64bit
                              integer := 512*8;
        IN_C_UNROLL         : --! @brief INPUT  CHANNEL UNROLL SIZE :
                              integer := 1;
        OUT_C_UNROLL        : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                              integer := 8;
        DATA_ADDR_WIDTH     : --! @brief DATA ADDRESS WIDTH :
                              --! IN_DATA/OUT_DATA/K_DATA/TH_DATA のメモリアドレスのビット幅を指定する.
                              integer := 32;
        S_AXI_ADDR_WIDTH    : --! @brief CSR I/F AXI ADDRRESS WIDTH :
                              integer := 32;
        S_AXI_DATA_WIDTH    : --! @brief CSR I/F AXI DATA WIDTH :
                              integer := 32;
        S_AXI_ID_WIDTH      : --! @brief CSR I/F AXI4 ID WIDTH :
                              integer := 4;
        I_AXI_ADDR_WIDTH    : --! @brief IN  DATA AXI ADDRESS WIDTH :
                              integer := 32;
        I_AXI_DATA_WIDTH    : --! @brief IN  DATA AXI DATA WIDTH :
                              integer := 64;
        I_AXI_ID_WIDTH      : --! @brief IN  DATA AXI ID WIDTH :
                              integer := 8;
        I_AXI_USER_WIDTH    : --! @brief IN  DATA AXI ID WIDTH :
                              integer := 8;
        I_AXI_XFER_SIZE     : --! @brief IN  DATA AXI MAX XFER_SIZE :
                              integer := 11;
        I_AXI_ID            : --! @brief IN  DATA AXI ID :
                              integer := 0;
        I_AXI_PROT          : --! @brief IN  DATA AXI PROT :
                              integer := 1;
        I_AXI_QOS           : --! @brief IN  DATA AXI QOS :
                              integer := 0;
        I_AXI_REGION        : --! @brief IN  DATA AXI REGION :
                              integer := 0;
        I_AXI_CACHE         : --! @brief IN  DATA AXI REGION :
                              integer := 15;
        I_AXI_REQ_QUEUE     : --! @brief IN  DATA AXI REQUEST QUEUE SIZE :
                              integer := 4;
        O_AXI_ADDR_WIDTH    : --! @brief OUT DATA AXI ADDRESS WIDTH :
                              integer := 32;
        O_AXI_DATA_WIDTH    : --! @brief OUT DATA AXI DATA WIDTH :
                              integer := 64;
        O_AXI_ID_WIDTH      : --! @brief OUT DATA AXI ID WIDTH :
                              integer := 8;
        O_AXI_USER_WIDTH    : --! @brief OUT DATA AXI ID WIDTH :
                              integer := 8;
        O_AXI_XFER_SIZE     : --! @brief OUT DATA AXI MAX XFER_SIZE :
                              integer := 11;
        O_AXI_ID            : --! @brief OUT DATA AXI ID :
                              integer := 0;
        O_AXI_PROT          : --! @brief OUT DATA AXI PROT :
                              integer := 1;
        O_AXI_QOS           : --! @brief OUT DATA AXI QOS :
                              integer := 0;
        O_AXI_REGION        : --! @brief OUT DATA AXI REGION :
                              integer := 0;
        O_AXI_CACHE         : --! @brief OUT DATA AXI REGION :
                              integer := 15;
        O_AXI_REQ_QUEUE     : --! @brief OUT DATA AXI REQUEST QUEUE SIZE :
                              integer := 4;
        K_AXI_ADDR_WIDTH    : --! @brief K   DATA AXI ADDRESS WIDTH :
                              integer := 32;
        K_AXI_DATA_WIDTH    : --! @brief K   DATA AXI DATA WIDTH :
                              integer := 64;
        K_AXI_ID_WIDTH      : --! @brief K   DATA AXI ID WIDTH :
                              integer := 8;
        K_AXI_USER_WIDTH    : --! @brief K   DATA AXI ID WIDTH :
                              integer := 8;
        K_AXI_XFER_SIZE     : --! @brief K   DATA AXI MAX XFER_SIZE :
                              integer := 11;
        K_AXI_ID            : --! @brief K   DATA AXI ID :
                              integer := 0;
        K_AXI_PROT          : --! @brief K   DATA AXI PROT :
                              integer := 1;
        K_AXI_QOS           : --! @brief K   DATA AXI QOS :
                              integer := 0;
        K_AXI_REGION        : --! @brief K   DATA AXI REGION :
                              integer := 0;
        K_AXI_CACHE         : --! @brief K   DATA AXI REGION :
                              integer := 15;
        K_AXI_REQ_QUEUE     : --! @brief K   DATA AXI REQUEST QUEUE SIZE :
                              integer := 4;
        T_AXI_ADDR_WIDTH    : --! @brief TH  DATA AXI ADDRESS WIDTH :
                              integer := 32;
        T_AXI_DATA_WIDTH    : --! @brief TH  DATA AXI DATA WIDTH :
                              integer := 64;
        T_AXI_ID_WIDTH      : --! @brief TH  DATA AXI ID WIDTH :
                              integer := 8;
        T_AXI_USER_WIDTH    : --! @brief TH  DATA AXI ID WIDTH :
                              integer := 8;
        T_AXI_XFER_SIZE     : --! @brief TH  DATA AXI MAX XFER_SIZE :
                              integer := 11;
        T_AXI_ID            : --! @brief TH  DATA AXI ID :
                              integer := 0;
        T_AXI_PROT          : --! @brief TH  DATA AXI PROT :
                              integer := 1;
        T_AXI_QOS           : --! @brief TH  DATA AXI QOS :
                              integer := 0;
        T_AXI_REGION        : --! @brief TH  DATA AXI REGION :
                              integer := 0;
        T_AXI_CACHE         : --! @brief TH  DATA AXI REGION :
                              integer := 15;
        T_AXI_REQ_QUEUE     : --! @brief TH  DATA AXI REQUEST QUEUE SIZE :
                              integer := 1
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        ACLK                : in  std_logic;
        ARESETn             : in  std_logic;
    -------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        S_AXI_ARID          : in  std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_ARADDR        : in  std_logic_vector(S_AXI_ADDR_WIDTH  -1 downto 0);
        S_AXI_ARLEN         : in  std_logic_vector(7 downto 0);
        S_AXI_ARSIZE        : in  std_logic_vector(2 downto 0);
        S_AXI_ARBURST       : in  std_logic_vector(1 downto 0);
        S_AXI_ARVALID       : in  std_logic;
        S_AXI_ARREADY       : out std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Read Data Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_RID           : out std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_RDATA         : out std_logic_vector(S_AXI_DATA_WIDTH  -1 downto 0);
        S_AXI_RRESP         : out std_logic_vector(1 downto 0);  
        S_AXI_RLAST         : out std_logic;
        S_AXI_RVALID        : out std_logic;
        S_AXI_RREADY        : in  std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Write Address Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_AWID          : in  std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_AWADDR        : in  std_logic_vector(S_AXI_ADDR_WIDTH  -1 downto 0);
        S_AXI_AWLEN         : in  std_logic_vector(7 downto 0);
        S_AXI_AWSIZE        : in  std_logic_vector(2 downto 0);
        S_AXI_AWBURST       : in  std_logic_vector(1 downto 0);
        S_AXI_AWVALID       : in  std_logic;
        S_AXI_AWREADY       : out std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Write Data Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_WDATA         : in  std_logic_vector(S_AXI_DATA_WIDTH  -1 downto 0);
        S_AXI_WSTRB         : in  std_logic_vector(S_AXI_DATA_WIDTH/8-1 downto 0);
        S_AXI_WLAST         : in  std_logic;
        S_AXI_WVALID        : in  std_logic;
        S_AXI_WREADY        : out std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Write Response Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_BID           : out std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_BRESP         : out std_logic_vector(1 downto 0);
        S_AXI_BVALID        : out std_logic;
        S_AXI_BREADY        : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_ARID         : out std_logic_vector(I_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_ARADDR       : out std_logic_vector(I_AXI_ADDR_WIDTH  -1 downto 0);
        IO_AXI_ARLEN        : out std_logic_vector(7 downto 0);
        IO_AXI_ARSIZE       : out std_logic_vector(2 downto 0);
        IO_AXI_ARBURST      : out std_logic_vector(1 downto 0);
        IO_AXI_ARLOCK       : out std_logic_vector(0 downto 0);
        IO_AXI_ARCACHE      : out std_logic_vector(3 downto 0);
        IO_AXI_ARPROT       : out std_logic_vector(2 downto 0);
        IO_AXI_ARQOS        : out std_logic_vector(3 downto 0);
        IO_AXI_ARREGION     : out std_logic_vector(3 downto 0);
        IO_AXI_ARUSER       : out std_logic_vector(I_AXI_USER_WIDTH  -1 downto 0);
        IO_AXI_ARVALID      : out std_logic;
        IO_AXI_ARREADY      : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_RID          : in  std_logic_vector(I_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_RDATA        : in  std_logic_vector(I_AXI_DATA_WIDTH  -1 downto 0);
        IO_AXI_RRESP        : in  std_logic_vector(1 downto 0);
        IO_AXI_RLAST        : in  std_logic;
        IO_AXI_RVALID       : in  std_logic;
        IO_AXI_RREADY       : out std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_AWID         : out std_logic_vector(O_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_AWADDR       : out std_logic_vector(O_AXI_ADDR_WIDTH  -1 downto 0);
        IO_AXI_AWLEN        : out std_logic_vector(7 downto 0);
        IO_AXI_AWSIZE       : out std_logic_vector(2 downto 0);
        IO_AXI_AWBURST      : out std_logic_vector(1 downto 0);
        IO_AXI_AWLOCK       : out std_logic_vector(0 downto 0);
        IO_AXI_AWCACHE      : out std_logic_vector(3 downto 0);
        IO_AXI_AWPROT       : out std_logic_vector(2 downto 0);
        IO_AXI_AWQOS        : out std_logic_vector(3 downto 0);
        IO_AXI_AWREGION     : out std_logic_vector(3 downto 0);
        IO_AXI_AWUSER       : out std_logic_vector(O_AXI_USER_WIDTH  -1 downto 0);
        IO_AXI_AWVALID      : out std_logic;
        IO_AXI_AWREADY      : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_WID          : out std_logic_vector(O_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_WDATA        : out std_logic_vector(O_AXI_DATA_WIDTH  -1 downto 0);
        IO_AXI_WSTRB        : out std_logic_vector(O_AXI_DATA_WIDTH/8-1 downto 0);
        IO_AXI_WLAST        : out std_logic;
        IO_AXI_WVALID       : out std_logic;
        IO_AXI_WREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_BID          : in  std_logic_vector(O_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_BRESP        : in  std_logic_vector(1 downto 0);
        IO_AXI_BVALID       : in  std_logic;
        IO_AXI_BREADY       : out std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_ARID          : out std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_ARADDR        : out std_logic_vector(K_AXI_ADDR_WIDTH  -1 downto 0);
        K_AXI_ARLEN         : out std_logic_vector(7 downto 0);
        K_AXI_ARSIZE        : out std_logic_vector(2 downto 0);
        K_AXI_ARBURST       : out std_logic_vector(1 downto 0);
        K_AXI_ARLOCK        : out std_logic_vector(0 downto 0);
        K_AXI_ARCACHE       : out std_logic_vector(3 downto 0);
        K_AXI_ARPROT        : out std_logic_vector(2 downto 0);
        K_AXI_ARQOS         : out std_logic_vector(3 downto 0);
        K_AXI_ARREGION      : out std_logic_vector(3 downto 0);
        K_AXI_ARUSER        : out std_logic_vector(K_AXI_USER_WIDTH  -1 downto 0);
        K_AXI_ARVALID       : out std_logic;
        K_AXI_ARREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_RID           : in  std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_RDATA         : in  std_logic_vector(K_AXI_DATA_WIDTH  -1 downto 0);
        K_AXI_RRESP         : in  std_logic_vector(1 downto 0);
        K_AXI_RLAST         : in  std_logic;
        K_AXI_RVALID        : in  std_logic;
        K_AXI_RREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_AWID          : out std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_AWADDR        : out std_logic_vector(K_AXI_ADDR_WIDTH  -1 downto 0);
        K_AXI_AWLEN         : out std_logic_vector(7 downto 0);
        K_AXI_AWSIZE        : out std_logic_vector(2 downto 0);
        K_AXI_AWBURST       : out std_logic_vector(1 downto 0);
        K_AXI_AWLOCK        : out std_logic_vector(0 downto 0);
        K_AXI_AWCACHE       : out std_logic_vector(3 downto 0);
        K_AXI_AWPROT        : out std_logic_vector(2 downto 0);
        K_AXI_AWQOS         : out std_logic_vector(3 downto 0);
        K_AXI_AWREGION      : out std_logic_vector(3 downto 0);
        K_AXI_AWUSER        : out std_logic_vector(K_AXI_USER_WIDTH  -1 downto 0);
        K_AXI_AWVALID       : out std_logic;
        K_AXI_AWREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_WID           : out std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_WDATA         : out std_logic_vector(K_AXI_DATA_WIDTH  -1 downto 0);
        K_AXI_WSTRB         : out std_logic_vector(K_AXI_DATA_WIDTH/8-1 downto 0);
        K_AXI_WLAST         : out std_logic;
        K_AXI_WVALID        : out std_logic;
        K_AXI_WREADY        : in  std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_BID           : in  std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_BRESP         : in  std_logic_vector(1 downto 0);
        K_AXI_BVALID        : in  std_logic;
        K_AXI_BREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_ARID          : out std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_ARADDR        : out std_logic_vector(T_AXI_ADDR_WIDTH  -1 downto 0);
        T_AXI_ARLEN         : out std_logic_vector(7 downto 0);
        T_AXI_ARSIZE        : out std_logic_vector(2 downto 0);
        T_AXI_ARBURST       : out std_logic_vector(1 downto 0);
        T_AXI_ARLOCK        : out std_logic_vector(0 downto 0);
        T_AXI_ARCACHE       : out std_logic_vector(3 downto 0);
        T_AXI_ARPROT        : out std_logic_vector(2 downto 0);
        T_AXI_ARQOS         : out std_logic_vector(3 downto 0);
        T_AXI_ARREGION      : out std_logic_vector(3 downto 0);
        T_AXI_ARUSER        : out std_logic_vector(K_AXI_USER_WIDTH  -1 downto 0);
        T_AXI_ARVALID       : out std_logic;
        T_AXI_ARREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_RID           : in  std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_RDATA         : in  std_logic_vector(T_AXI_DATA_WIDTH  -1 downto 0);
        T_AXI_RRESP         : in  std_logic_vector(1 downto 0);
        T_AXI_RLAST         : in  std_logic;
        T_AXI_RVALID        : in  std_logic;
        T_AXI_RREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_AWID          : out std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_AWADDR        : out std_logic_vector(T_AXI_ADDR_WIDTH  -1 downto 0);
        T_AXI_AWLEN         : out std_logic_vector(7 downto 0);
        T_AXI_AWSIZE        : out std_logic_vector(2 downto 0);
        T_AXI_AWBURST       : out std_logic_vector(1 downto 0);
        T_AXI_AWLOCK        : out std_logic_vector(0 downto 0);
        T_AXI_AWCACHE       : out std_logic_vector(3 downto 0);
        T_AXI_AWPROT        : out std_logic_vector(2 downto 0);
        T_AXI_AWQOS         : out std_logic_vector(3 downto 0);
        T_AXI_AWREGION      : out std_logic_vector(3 downto 0);
        T_AXI_AWUSER        : out std_logic_vector(T_AXI_USER_WIDTH  -1 downto 0);
        T_AXI_AWVALID       : out std_logic;
        T_AXI_AWREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_WID           : out std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_WDATA         : out std_logic_vector(T_AXI_DATA_WIDTH  -1 downto 0);
        T_AXI_WSTRB         : out std_logic_vector(T_AXI_DATA_WIDTH/8-1 downto 0);
        T_AXI_WLAST         : out std_logic;
        T_AXI_WVALID        : out std_logic;
        T_AXI_WREADY        : in  std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_BID           : in  std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_BRESP         : in  std_logic_vector(1 downto 0);
        T_AXI_BVALID        : in  std_logic;
        T_AXI_BREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- Interrupt Request
    -------------------------------------------------------------------------------
        IRQ                 : out std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_CORE                                                      --
-----------------------------------------------------------------------------------
component QCONV_STRIP_CORE
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        IN_BUF_SIZE     : --! @brief IN DATA BUFFER SIZE :
                          --! 入力バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --!   = 64 bit.
                          --! * 入力バッファの容量は 入力チャネル × イメージの幅.
                          integer := 512*4*1;  -- 512word × BANK_SIZE × IN_C_UNROLL 
        K_BUF_SIZE      : --! @brief K DATA BUFFER SIZE :
                          --! カーネル係数バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは 3 * 3 * QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                          integer := 512*3*3*16*1;  -- 512word × 3 × 3 × OUT_C_UNROLL × IN_C_UNROLL
        TH_BUF_SIZE     : --! @brief THRESHOLDS DATA BUFFER SIZE :
                          --! THRESHOLDS バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS
                          --! * = 64bit
                          integer := 512*16;
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 16;
        OUT_DATA_BITS   : --! @brief OUTPUT DATA BIT SIZE :
                          --! OUT_DATA のビット幅を指定する.
                          --! * OUT_DATA のビット幅は、64の倍数でなければならない.
                          integer := 64
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : --! @brief INPUT C CHANNEL SIZE :
                          in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        IN_W            : --! @brief INPUT IMAGE WIDTH :
                          in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        IN_H            : --! @brief INPUT IMAGE HEIGHT :
                          in  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        K_W             : --! @brief KERNEL WIDTH :
                          in  std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        K_H             : --! @brief KERNEL HEIGHT :
                          in  std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        LEFT_PAD_SIZE   : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        RIGHT_PAD_SIZE  : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        TOP_PAD_SIZE    : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        BOTTOM_PAD_SIZE : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        USE_TH          : --! @brief USE THRESHOLD REGISTER :
                          in  std_logic;
        PARAM_IN        : --! @brief K DATA / TH DATA INPUT FLAG :
                          in  std_logic;
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- データ入力 I/F
    -------------------------------------------------------------------------------
        IN_DATA         : --! @brief INPUT IN_DATA :
                          --! IN_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        IN_VALID        : --! @brief INPUT IN_DATA VALID :
                          --! IN_DATA 入力有効信号.
                          in  std_logic;
        IN_READY        : --! @brief INPUT IN_DATA READY :
                          --! IN_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- カーネル係数入力 I/F
    -------------------------------------------------------------------------------
        K_DATA          : --! @brief INPUT K_DATA :
                          --! K_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        K_VALID         : --! @brief INPUT K_DATA VALID :
                          --! K_DATA 入力有効信号.
                          in  std_logic;
        K_READY         : --! @brief INPUT K_DATA READY :
                          --! K_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- スレッシュホールド係数入力 I/F
    -------------------------------------------------------------------------------
        TH_DATA         : --! @brief INPUT TH_DATA :
                          --! TH_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS-1 downto 0);
        TH_VALID        : --! @brief INPUT TH_DATA VALID :
                          --! TH_DATA 入力有効信号.
                          in  std_logic;
        TH_READY        : --! @brief INPUT TH_DATA READY :
                          --! TH_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- データ出力 I/F
    -------------------------------------------------------------------------------
        OUT_DATA        : --! @brief OUTPUT DATA :
                          --! OUT DATA 出力.
                          out std_logic_vector(OUT_DATA_BITS-1 downto 0);
        OUT_LAST        : --! @brief OUTPUT LAST DATA :
                          --! OUT LAST 出力.
                          out std_logic;
        OUT_VALID       : --! @brief OUT_DATA VALID :
                          --! OUT_DATA 出力有効信号.
                          out std_logic;
        OUT_READY       : --! @brief OUT_DATA READY :
                          --! OUT_DATA レディ信号.
                          in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_CONTROLLER                                                --
-----------------------------------------------------------------------------------
component QCONV_STRIP_CONTROLLER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        IN_BUF_SIZE     : --! @brief IN DATA BUFFER SIZE :
                          --! 入力バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --!   = 64 bit.
                          --! * 入力バッファの容量は 入力チャネル × イメージの幅.
                          integer := 512*4*1;  -- 512word × BANK_SIZE × IN_C_UNROLL 
        K_BUF_SIZE      : --! @brief K DATA BUFFER SIZE :
                          --! カーネル係数バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは 3 * 3 * QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                          integer := 512*3*3*16*1;  -- 512word × 3 × 3 × OUT_C_UNROLL × IN_C_UNROLL
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1
    );
    port(
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Register Interface
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        IN_W            : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        IN_H            : in  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        OUT_C           : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        OUT_W           : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        OUT_H           : in  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        K_W             : in  std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        K_H             : in  std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        PAD_SIZE        : in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        USE_TH          : in  std_logic;
        REQ_VALID       : in  std_logic;
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_READY       : in  std_logic;
        RES_STATUS      : out std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Core Module Interface
    -------------------------------------------------------------------------------
        CORE_IN_C       : out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        CORE_IN_W       : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        CORE_IN_H       : out std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        CORE_OUT_C      : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        CORE_OUT_W      : out std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        CORE_OUT_H      : out std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        CORE_K_W        : out std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        CORE_K_H        : out std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        CORE_L_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_R_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_T_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_B_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_USE_TH     : out std_logic;
        CORE_PARAM_IN   : out std_logic;
        CORE_REQ_VALID  : out std_logic;
        CORE_REQ_READY  : in  std_logic;
        CORE_RES_VALID  : in  std_logic;
        CORE_RES_READY  : out std_logic;
        CORE_RES_STATUS : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In Data AXI Reader Module Interface
    -------------------------------------------------------------------------------
        I_IN_C          : out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        I_IN_W          : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_IN_H          : out std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        I_X_POS         : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_X_SIZE        : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_REQ_VALID     : out std_logic;
        I_REQ_READY     : in  std_logic;
        I_RES_VALID     : in  std_logic;
        I_RES_READY     : out std_logic;
        I_RES_NONE      : in  std_logic;
        I_RES_ERROR     : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Kernel Weight Data AXI Reader Module Interface
    -------------------------------------------------------------------------------        
        K_IN_C          : out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        K_OUT_C         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_OUT_C_POS     : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_OUT_C_SIZE    : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_REQ_K3x3      : out std_logic;
        K_REQ_VALID     : out std_logic;
        K_REQ_READY     : in  std_logic;
        K_RES_VALID     : in  std_logic;
        K_RES_READY     : out std_logic;
        K_RES_NONE      : in  std_logic;
        K_RES_ERROR     : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Thresholds Data AXI Reader Module Interface
    -------------------------------------------------------------------------------        
        T_OUT_C         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        T_OUT_C_POS     : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        T_OUT_C_SIZE    : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        T_REQ_VALID     : out std_logic;
        T_REQ_READY     : in  std_logic;
        T_RES_VALID     : in  std_logic;
        T_RES_READY     : out std_logic;
        T_RES_NONE      : in  std_logic;
        T_RES_ERROR     : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Data AXI Writer Module Interface
    -------------------------------------------------------------------------------
        O_OUT_C         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        O_OUT_W         : out std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        O_OUT_H         : out std_logic_vector(QCONV_PARAM.OUT_H_BITS-1 downto 0);
        O_C_POS         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        O_C_SIZE        : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        O_X_POS         : out std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        O_X_SIZE        : out std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        O_USE_TH        : out std_logic;
        O_REQ_VALID     : out std_logic;
        O_REQ_READY     : in  std_logic;
        O_RES_VALID     : in  std_logic;
        O_RES_READY     : out std_logic;
        O_RES_NONE      : in  std_logic;
        O_RES_ERROR     : in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_REGISTERS                                                 --
-----------------------------------------------------------------------------------
component QCONV_STRIP_REGISTERS
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        ID              : --! @brief REGISTER ID STRING :
                          string(1 to 8) := "QCONV-S1";
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        DATA_ADDR_WIDTH : --! @brief I_DATA_ADDR/K_DATA_ADDR/T_DATA_ADDR/O_DATA_ADDR WIDTH :
                          integer := 64;
        REGS_ADDR_WIDTH : --! @brief REGISTER ADDRESS WIDTH :
                          --! レジスタアクセスインターフェースのアドレスのビット数.
                          integer := 7;
        REGS_DATA_WIDTH : --! @brief REGISTER ADDRESS WIDTH :
                          --! レジスタアクセスインターフェースのデータのビット数.
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- Register Access Interface
    -------------------------------------------------------------------------------
        REGS_REQ        : --! @brief REGISTER ACCESS REQUEST :
                          --! レジスタアクセス要求信号.
                          in  std_logic;
        REGS_WRITE      : --! @brief REGISTER WRITE ACCESS :
                          --! レジスタライトアクセス信号.
                          --! * この信号が'1'の時はライトアクセスを行う.
                          --! * この信号が'0'の時はリードアクセスを行う.
                          in  std_logic;
        REGS_ADDR       : --! @brief REGISTER ACCESS ADDRESS :
                          --! レジスタアクセスアドレス信号.
                          in  std_logic_vector(REGS_ADDR_WIDTH  -1 downto 0);
        REGS_BEN        : --! @brief REGISTER BYTE ENABLE :
                          --! レジスタアクセスバイトイネーブル信号.
                          in  std_logic_vector(REGS_DATA_WIDTH/8-1 downto 0);
        REGS_WDATA      : --! @brief REGISTER ACCESS WRITE DATA :
                          --! レジスタアクセスライトデータ.
                          in  std_logic_vector(REGS_DATA_WIDTH  -1 downto 0);
        REGS_RDATA      : --! @brief REGISTER ACCESS READ DATA :
                          --! レジスタアクセスリードデータ.
                          out std_logic_vector(REGS_DATA_WIDTH  -1 downto 0);
        REGS_ACK        : --! @brief REGISTER ACCESS ACKNOWLEDGE :
                          --! レジスタアクセス応答信号.
                          out std_logic;
        REGS_ERR        : --! @brief REGISTER ACCESS ERROR ACKNOWLEDGE :
                          --! レジスタアクセスエラー応答信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Registers
    -------------------------------------------------------------------------------
        I_DATA_ADDR     : --! @brief IN  DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        O_DATA_ADDR     : --! @brief OUT DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        K_DATA_ADDR     : --! @brief K   DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        T_DATA_ADDR     : --! @brief TH  DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        I_WIDTH         : --! @brief IN  WIDTH REGISTER :
                          out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_HEIGHT        : --! @brief IN  HEIGHT REGISTER :
                          out std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        I_CHANNELS      : --! @brief IN  CHANNELS REGISTER :
                          out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        O_WIDTH         : --! @brief OUT WIDTH REGISTER :
                          out std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        O_HEIGHT        : --! @brief OUT HEIGHT REGISTER :
                          out std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        O_CHANNELS      : --! @brief OUT CHANNELS REGISTER :
                          out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_WIDTH         : --! @brief K   WIDTH REGISTER :
                          out std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        K_HEIGHT        : --! @brief K   HEIGHT REGISTER :
                          out std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        PAD_SIZE        : --! @brief PAD SIZE REGISTER :
                          out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        USE_TH          : --! @brief USE THRESHOLD REGISTER :
                          out std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Request/Response Interface
    -------------------------------------------------------------------------------
        REQ_VALID       : --! @brief REQUEST VALID :
                          out std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          in  std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          in  std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          out std_logic;
        RES_STATUS      : --! @brief RESPONSE STATUS :
                          in  std_logic;
        REQ_RESET       : --! @brief RESET REQUEST :
                          out std_logic;
        REQ_STOP        : --! @brief STOP REQUEST :
                          out std_logic;
        REQ_PAUSE       : --! @brief PAUSE REQUEST :
                          out std_logic;
    -------------------------------------------------------------------------------
    -- Interrupt Request 
    -------------------------------------------------------------------------------
        IRQ             : --! @brief Interrupt Request :
                          out std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_IN_DATA_BUFFER                                            --
-----------------------------------------------------------------------------------
component QCONV_STRIP_IN_DATA_BUFFER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        O_PARAM         : --! @brief OUTPUT STREAM PARAMETER :
                          --! 出力側の IMAGE STREAM のパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                              ELEM_BITS => 64,
                              C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1*3*3*32),
                              D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                          );
        I_SHAPE         : --! @brief INPUT  SHAPE :
                          --! 入力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        O_SHAPE         : --! @brief OUTPUT SHAPE :
                          --! 出力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        ELEMENT_SIZE    : --! @brief ELEMENT SIZE :
                          --! 列方向の要素数を指定する.
                          integer := 256;
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 1;
        ID              : --! @brief SDPRAM IDENTIFIER :
                          --! どのモジュールで使われているかを示す識別番号.
                          integer := 0 
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : --! @brief INPUT C CHANNEL SIZE :
                          in  integer range 0 to I_SHAPE.C.MAX_SIZE := I_SHAPE.C.SIZE;
        IN_W            : --! @brief INPUT IMAGE WIDTH :
                          in  integer range 0 to I_SHAPE.X.MAX_SIZE := I_SHAPE.X.SIZE;
        IN_H            : --! @brief INPUT IMAGE HEIGHT :
                          in  integer range 0 to I_SHAPE.Y.MAX_SIZE := I_SHAPE.Y.SIZE;
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  integer range 0 to O_SHAPE.C.MAX_SIZE := O_SHAPE.C.SIZE;
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  integer range 0 to O_SHAPE.X.MAX_SIZE := O_SHAPE.X.SIZE;
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  integer range 0 to O_SHAPE.Y.MAX_SIZE := O_SHAPE.Y.SIZE;
        K3x3            : --! @brief KERNEL SIZE :
                          --! * Kernel が 3x3 の場合は'1'.
                          --! * Kernel が 1x1 の場合は'0'.
                          in  std_logic;
        LEFT_PAD_SIZE   : --! @brief IMAGE WIDTH START PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        RIGHT_PAD_SIZE  : --! @brief IMAGE WIDTH LAST  PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        TOP_PAD_SIZE    : --! @brief IMAGE HEIGHT START PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        BOTTOM_PAD_SIZE : --! @brief IMAGE HEIGHT LAST  PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT IN_DATA :
                          --! IN_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        I_VALID         : --! @brief INPUT IN_DATA VALID :
                          --! IN_DATA 入力有効信号.
                          in  std_logic;
        I_READY         : --! @brief INPUT IN_DATA READY :
                          --! IN_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT IMAGE STREAM DATA :
                          --! ストリームデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT IMAGE STREAM DATA VALID :
                          --! 出力ストリームデータ有効信号.
                          out std_logic;
        O_READY         : --! @brief OUTPUT IMAGE STREAM DATA READY :
                          --! 出力ストリームデータレディ信号.
                          in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_TH_DATA_BUFFER                                            --
-----------------------------------------------------------------------------------
component QCONV_STRIP_TH_DATA_BUFFER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        O_PARAM         : --! @brief OUTPUT STREAM PARAMETER :
                          --! 出力側の IMAGE STREAM のパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                              ELEM_BITS => 64,
                              C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(3*3),
                              D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                          );
        O_SHAPE         : --! @brief OUTPUT SHAPE :
                          --! 出力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        ELEMENT_SIZE    : --! @brief ELEMENT SIZE :
                          --! THRESHOLDS バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS
                          --! * = 64bit
                          integer := 256;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 1;
        ID              : --! @brief SDPRAM IDENTIFIER :
                          --! どのモジュールで使われているかを示す識別番号.
                          integer := 0 
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  integer range 0 to O_SHAPE.C.MAX_SIZE := O_SHAPE.C.SIZE;
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  integer range 0 to O_SHAPE.X.MAX_SIZE := O_SHAPE.X.SIZE;
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  integer range 0 to O_SHAPE.Y.MAX_SIZE := O_SHAPE.Y.SIZE;
        REQ_WRITE       : --! @brief REQUEST BUFFER WRITE :
                          in  std_logic := '1';
        REQ_READ        : --! @brief REQUEST BUFFER READ :
                          in  std_logic := '1';
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT THRESHOLDS DATA :
                          --! THRESHOLDS DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS-1 downto 0);
        I_VALID         : --! @brief INPUT THRESHOLDS DATA VALID :
                          --! THRESHOLDS DATA 入力有効信号.
                          in  std_logic;
        I_READY         : --! @brief INPUT THRESHOLDS READY :
                          --! THRESHOLDS DATA 入力レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT THRESHOLDS DATA :
                          --! THRESHOLDS DATA 出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT THRESHOLDS DATA VALID :
                          --! THRESHOLDS DATA 出力有効信号.
                          out std_logic;
        O_READY         : --! @brief OUTPUT THRESHOLDS DATA READY :
                          --! THRESHOLDS DATA 出力レディ信号.
                          in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_K_DATA_BUFFER                                             --
-----------------------------------------------------------------------------------
component QCONV_STRIP_K_DATA_BUFFER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        O_PARAM         : --! @brief OUTPUT STREAM PARAMETER :
                          --! 出力側の IMAGE STREAM のパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                              ELEM_BITS => 32,
                              C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1*3*3),
                              D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                          );
        I_SHAPE         : --! @brief INPUT  SHAPE :
                          --! 入力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        O_SHAPE         : --! @brief OUTPUT SHAPE :
                          --! 出力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        ELEMENT_SIZE    : --! @brief ELEMENT SIZE :
                          --! カーネル係数バッファの容量を指定する.
                          --! * ここで指定する単位は9ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --! * 9ワードは 9 * 32 = 288 bit
                          --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                          integer := (1024/32)*256;
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 1;
        QUEUE_SIZE      : --! @brief OUTPUT PIPELINE QUEUE SIZE :
                          --! パイプラインレジスタの深さを指定する.
                          --! * QUEUE_SIZE=0 の場合は出力にキューが挿入されずダイレ
                          --!   クトに出力される.
                          integer := 0;
        ID              : --! @brief SDPRAM IDENTIFIER :
                          --! どのモジュールで使われているかを示す識別番号.
                          integer := 0 
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : --! @brief INPUT C CHANNEL SIZE :
                          in  integer range 0 to I_SHAPE.C.MAX_SIZE := I_SHAPE.C.SIZE;
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  integer range 0 to O_SHAPE.C.MAX_SIZE := O_SHAPE.C.SIZE;
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  integer range 0 to O_SHAPE.X.MAX_SIZE := O_SHAPE.X.SIZE;
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  integer range 0 to O_SHAPE.Y.MAX_SIZE := O_SHAPE.Y.SIZE;
        K3x3            : --! @brief KERNEL SIZE :
                          --! * Kernel が 3x3 の場合は'1'.
                          --! * Kernel が 1x1 の場合は'0'.
                          in  std_logic;
        REQ_WRITE       : --! @brief REQUEST BUFFER WRITE :
                          in  std_logic := '1';
        REQ_READ        : --! @brief REQUEST BUFFER READ :
                          in  std_logic := '1';
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT K_DATA :
                          --! K_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        I_VALID         : --! @brief INPUT K_DATA VALID :
                          --! K_DATA 入力有効信号.
                          in  std_logic;
        I_READY         : --! @brief INPUT IN_DATA READY :
                          --! K_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT IMAGE STREAM DATA :
                          --! ストリームデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT IMAGE STREAM DATA VALID :
                          --! 出力ストリームデータ有効信号.
                          out std_logic;
        O_READY         : --! @brief OUTPUT IMAGE STREAM DATA READY :
                          --! 出力ストリームデータレディ信号.
                          in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_MULTIPLIER                                                      --
-----------------------------------------------------------------------------------
component QCONV_MULTIPLIER
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        I_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE IMAGE DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        K_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE WEIGHT DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        O_PARAM         : --! @brief OUTPUT CONVOLUTION PIPELINE DATA PARAMETER :
                          --! パイプラインデータ出力ポートのパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        CHECK_K_VALID   : --! @brief CHECK K VALID :
                          --! K 入力の VALID フラグをチェックするか否かを指定する.
                          --! * CHECK_K_VALID=1の場合はチェックする.その分、少し回路
                          --!   が大きくなるかも.
                          --! * CHECK_K_VALID=0の場合はチェックしない.その分、少し回
                          --!   路が小さくなるかも.
                          integer := 1;
        QUEUE_SIZE      : --! @brief PIPELINE QUEUE SIZE :
                          --! パイプラインレジスタの深さを指定する.
                          --! * QUEUE_SIZE=0 の場合は出力にキューが挿入されずダイレ
                          --!   クトに出力される.
                          integer := 2
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(I_PARAM.DATA.SIZE-1 downto 0);
        I_VALID         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * I_DATAが有効であることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        I_READY         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
        K_DATA          : --! @brief INPUT CONVOLUTION PIPELINE WEIGHT DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(K_PARAM.DATA.SIZE-1 downto 0);
        K_VALID         : --! @brief INPUT CONVOLUTION PIPELINE WEIGHT DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * K_DATAが有効であることを示す.
                          --! * K_VALID='1'and K_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        K_READY         : --! @brief INPUT CONVOLUTION PIPELINE WEIGHT DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * K_VALID='1'and K_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 出力パイプラインデータ有効信号.
                          --! * O_DATA が有効であることを示す.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          out std_logic;
        O_READY         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 出力パイプラインデータレディ信号.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_APPLY_THRESHOLDS                                                --
-----------------------------------------------------------------------------------
component QCONV_APPLY_THRESHOLDS
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        I_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE IMAGE DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          --! * 次の条件を満していなければならない.
                          --!     I_PARAM.SHAPE = O_PARAM.SHAPE
                          --!     I_PARAM.SHAPE = K_PARAM.SHAPE
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        T_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE THRESHOLD DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          --! * 次の条件を満していなければならない.
                          --!     T_PARAM.SHAPE = I_PARAM.SHAPE
                          --!     T_PARAM.SHAPE = O_PARAM.SHAPE
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        O_PARAM         : --! @brief OUTPUT CONVOLUTION PIPELINE DATA PARAMETER :
                          --! パイプラインデータ出力ポートのパラメータを指定する.
                          --! * 次の条件を満していなければならない.
                          --!     O_PARAM.SHAPE = I_PARAM.SHAPE
                          --!     O_PARAM.SHAPE = T_PARAM.SHAPE
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        QUEUE_SIZE      : --! パイプラインレジスタの深さを指定する.
                          --! * QUEUE_SIZE=0 の場合は出力にキューが挿入されずダイレ
                          --!   クトに出力される.
                          integer := 2
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(I_PARAM.DATA.SIZE-1 downto 0);
        I_VALID         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * I_DATAが有効であることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        I_READY         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
        T_DATA          : --! @brief INPUT CONVOLUTION PIPELINE THRESHOLD DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(T_PARAM.DATA.SIZE-1 downto 0);
        T_VALID         : --! @brief INPUT CONVOLUTION PIPELINE THRESHOLD DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * T_DATAが有効であることを示す.
                          --! * T_VALID='1'and T_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        T_READY         : --! @brief INPUT CONVOLUTION PIPELINE THRESHOLD DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * T_VALID='1'and T_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 出力パイプラインデータ有効信号.
                          --! * O_DATA が有効であることを示す.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          out std_logic;
        O_READY         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 出力パイプラインデータレディ信号.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_K_DATA_AXI_READER                                         --
-----------------------------------------------------------------------------------
component QCONV_STRIP_K_DATA_AXI_READER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 128*(64/8);
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_ARID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_ARADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_ARLEN       : out std_logic_vector(7 downto 0);
        AXI_ARSIZE      : out std_logic_vector(2 downto 0);
        AXI_ARBURST     : out std_logic_vector(1 downto 0);
        AXI_ARLOCK      : out std_logic_vector(0 downto 0);
        AXI_ARCACHE     : out std_logic_vector(3 downto 0);
        AXI_ARPROT      : out std_logic_vector(2 downto 0);
        AXI_ARQOS       : out std_logic_vector(3 downto 0);
        AXI_ARREGION    : out std_logic_vector(3 downto 0);
        AXI_ARUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_ARVALID     : out std_logic;
        AXI_ARREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_RID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_RDATA       : in  std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_RRESP       : in  std_logic_vector(1 downto 0);
        AXI_RLAST       : in  std_logic;
        AXI_RVALID      : in  std_logic;
        AXI_RREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Master Interface.
    -------------------------------------------------------------------------------
        O_DATA          : out std_logic_vector(QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD -1 downto 0);
        O_LAST          : out std_logic;
        O_VALID         : out std_logic;
        O_READY         : in  std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_IN_C        : in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        REQ_OUT_C       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        REQ_OUT_C_POS   : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        REQ_OUT_C_SIZE  : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        REQ_K3x3        : in  std_logic;
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_IN_DATA_AXI_READER                                        --
-----------------------------------------------------------------------------------
component QCONV_STRIP_IN_DATA_AXI_READER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 12;
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_ARID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_ARADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_ARLEN       : out std_logic_vector(7 downto 0);
        AXI_ARSIZE      : out std_logic_vector(2 downto 0);
        AXI_ARBURST     : out std_logic_vector(1 downto 0);
        AXI_ARLOCK      : out std_logic_vector(0 downto 0);
        AXI_ARCACHE     : out std_logic_vector(3 downto 0);
        AXI_ARPROT      : out std_logic_vector(2 downto 0);
        AXI_ARQOS       : out std_logic_vector(3 downto 0);
        AXI_ARREGION    : out std_logic_vector(3 downto 0);
        AXI_ARUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_ARVALID     : out std_logic;
        AXI_ARREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_RID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_RDATA       : in  std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_RRESP       : in  std_logic_vector(1 downto 0);
        AXI_RLAST       : in  std_logic;
        AXI_RVALID      : in  std_logic;
        AXI_RREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Master Interface.
    -------------------------------------------------------------------------------
        O_DATA          : out std_logic_vector(QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        O_LAST          : out std_logic;
        O_VALID         : out std_logic;
        O_READY         : in  std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_IN_C        : in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        REQ_IN_W        : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        REQ_IN_H        : in  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        REQ_X_POS       : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        REQ_X_SIZE      : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_TH_DATA_AXI_READER                                        --
-----------------------------------------------------------------------------------
component QCONV_STRIP_TH_DATA_AXI_READER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 128*(64/8);
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_ARID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_ARADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_ARLEN       : out std_logic_vector(7 downto 0);
        AXI_ARSIZE      : out std_logic_vector(2 downto 0);
        AXI_ARBURST     : out std_logic_vector(1 downto 0);
        AXI_ARLOCK      : out std_logic_vector(0 downto 0);
        AXI_ARCACHE     : out std_logic_vector(3 downto 0);
        AXI_ARPROT      : out std_logic_vector(2 downto 0);
        AXI_ARQOS       : out std_logic_vector(3 downto 0);
        AXI_ARREGION    : out std_logic_vector(3 downto 0);
        AXI_ARUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_ARVALID     : out std_logic;
        AXI_ARREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_RID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_RDATA       : in  std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_RRESP       : in  std_logic_vector(1 downto 0);
        AXI_RLAST       : in  std_logic;
        AXI_RVALID      : in  std_logic;
        AXI_RREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Master Interface.
    -------------------------------------------------------------------------------
        O_DATA          : out std_logic_vector(QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS-1 downto 0);
        O_LAST          : out std_logic;
        O_VALID         : out std_logic;
        O_READY         : in  std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_OUT_C       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_OUT_C_POS   : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_OUT_C_SIZE  : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end component;
-----------------------------------------------------------------------------------
--! @brief QCONV_STRIP_OUT_DATA_AXI_WRITER                                       --
-----------------------------------------------------------------------------------
component QCONV_STRIP_OUT_DATA_AXI_WRITER
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 128*(64/8);
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        I_DATA_WIDTH    : --! @brief STREAM DATA WIDTH :
                          integer := 32;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_AWID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_AWADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_AWLEN       : out std_logic_vector(7 downto 0);
        AXI_AWSIZE      : out std_logic_vector(2 downto 0);
        AXI_AWBURST     : out std_logic_vector(1 downto 0);
        AXI_AWLOCK      : out std_logic_vector(0 downto 0);
        AXI_AWCACHE     : out std_logic_vector(3 downto 0);
        AXI_AWPROT      : out std_logic_vector(2 downto 0);
        AXI_AWQOS       : out std_logic_vector(3 downto 0);
        AXI_AWREGION    : out std_logic_vector(3 downto 0);
        AXI_AWUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_AWVALID     : out std_logic;
        AXI_AWREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_WID         : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_WDATA       : out std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_WSTRB       : out std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
        AXI_WLAST       : out std_logic;
        AXI_WVALID      : out std_logic;
        AXI_WREADY      : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        AXI_BID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_BRESP       : in  std_logic_vector(1 downto 0);
        AXI_BVALID      : in  std_logic;
        AXI_BREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Slave Interface.
    -------------------------------------------------------------------------------
        I_DATA          : in  std_logic_vector(I_DATA_WIDTH    -1 downto 0);
        I_STRB          : in  std_logic_vector(I_DATA_WIDTH/8  -1 downto 0) := (others => '1');
        I_LAST          : in  std_logic;
        I_VALID         : in  std_logic;
        I_READY         : out std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_OUT_C       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_OUT_W       : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        REQ_OUT_H       : in  std_logic_vector(QCONV_PARAM.OUT_H_BITS-1 downto 0);
        REQ_C_POS       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_C_SIZE      : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_X_POS       : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        REQ_X_SIZE      : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        REQ_USE_TH      : in  std_logic;
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end component;
end QCONV_COMPONENTS;
-----------------------------------------------------------------------------------
--!     @file    qconv_multiplier.vhd
--!     @brief   Quantized Convolution Multiplier Module
--!     @version 0.1.0
--!     @date    2019/4/27
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
-----------------------------------------------------------------------------------
--! @brief Quantized Convolution Multiplier Module
-----------------------------------------------------------------------------------
entity  QCONV_MULTIPLIER is
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        I_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE IMAGE DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        K_PARAM         : --! @brief INPUT  CONVOLUTION PIPELINE WEIGHT DATA PARAMETER :
                          --! パイプラインデータ入力ポートのパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        O_PARAM         : --! @brief OUTPUT CONVOLUTION PIPELINE DATA PARAMETER :
                          --! パイプラインデータ出力ポートのパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(8,1,1,1);
        CHECK_K_VALID   : --! @brief CHECK K VALID :
                          --! K 入力の VALID フラグをチェックするか否かを指定する.
                          --! * CHECK_K_VALID=1の場合はチェックする.その分、少し回路
                          --!   が大きくなるかも.
                          --! * CHECK_K_VALID=0の場合はチェックしない.その分、少し回
                          --!   路が小さくなるかも.
                          integer := 1;
        QUEUE_SIZE      : --! @brief PIPELINE QUEUE SIZE :
                          --! パイプラインレジスタの深さを指定する.
                          --! * QUEUE_SIZE=0 の場合は出力にキューが挿入されずダイレ
                          --!   クトに出力される.
                          integer := 2
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(I_PARAM.DATA.SIZE-1 downto 0);
        I_VALID         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * I_DATAが有効であることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        I_READY         : --! @brief INPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * I_VALID='1'and I_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
        K_DATA          : --! @brief INPUT CONVOLUTION PIPELINE WEIGHT DATA :
                          --! パイプラインデータ入力.
                          in  std_logic_vector(K_PARAM.DATA.SIZE-1 downto 0);
        K_VALID         : --! @brief INPUT CONVOLUTION PIPELINE WEIGHT DATA VALID :
                          --! 入力パイプラインデータ有効信号.
                          --! * K_DATAが有効であることを示す.
                          --! * K_VALID='1'and K_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          in  std_logic;
        K_READY         : --! @brief INPUT CONVOLUTION PIPELINE WEIGHT DATA READY :
                          --! 入力パイプラインデータレディ信号.
                          --! * 次のパイプラインデータを入力出来ることを示す.
                          --! * K_VALID='1'and K_READY='1'でパイプラインデータが
                          --!   取り込まれる.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA :
                          --! パイプラインデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA VALID :
                          --! 出力パイプラインデータ有効信号.
                          --! * O_DATA が有効であることを示す.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          out std_logic;
        O_READY         : --! @brief OUTPUT CONVOLUTION PIPELINE IMAGE DATA READY :
                          --! 出力パイプラインデータレディ信号.
                          --! * O_VALID='1'and O_READY='1'でパイプラインデータが
                          --!   キューから取り除かれる.
                          in  std_logic
    );
end QCONV_MULTIPLIER;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.COMPONENTS.PIPELINE_REGISTER;
architecture RTL of QCONV_MULTIPLIER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    subtype   I_ELEM_TYPE     is std_logic_vector(I_PARAM.ELEM_BITS-1 downto 0);
    type      I_ELEM_VECTOR   is array(0 to I_PARAM.SHAPE.Y.SIZE-1,
                                       0 to I_PARAM.SHAPE.X.SIZE-1,
                                       0 to I_PARAM.SHAPE.D.SIZE-1,
                                       0 to I_PARAM.SHAPE.C.SIZE-1) of I_ELEM_TYPE;
    signal    i_element       :  I_ELEM_VECTOR;
    signal    i_c_atrb        :  IMAGE_STREAM_ATRB_VECTOR(0 to I_PARAM.SHAPE.C.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    subtype   K_ELEM_TYPE     is std_logic_vector(K_PARAM.ELEM_BITS-1 downto 0);
    type      K_ELEM_VECTOR   is array(0 to K_PARAM.SHAPE.Y.SIZE-1,
                                       0 to K_PARAM.SHAPE.X.SIZE-1,
                                       0 to K_PARAM.SHAPE.D.SIZE-1,
                                       0 to K_PARAM.SHAPE.C.SIZE-1) of K_ELEM_TYPE;
    signal    k_element       :  K_ELEM_VECTOR;
    signal    k_c_atrb        :  IMAGE_STREAM_ATRB_VECTOR(0 to K_PARAM.SHAPE.C.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  Q_PARAM         :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => O_PARAM.ELEM_BITS * QCONV_PARAM.NBITS_PER_WORD,
                                          C         => I_PARAM.SHAPE.C,
                                          D         => I_PARAM.SHAPE.D,
                                          X         => I_PARAM.SHAPE.X,
                                          Y         => I_PARAM.SHAPE.Y
                                      );
    subtype   Q_ELEM_TYPE     is std_logic_vector(Q_PARAM.ELEM_BITS-1 downto 0);
    type      Q_ELEM_VECTOR   is array(0 to Q_PARAM.SHAPE.Y.SIZE-1,
                                       0 to Q_PARAM.SHAPE.X.SIZE-1,
                                       0 to Q_PARAM.SHAPE.D.SIZE-1,
                                       0 to Q_PARAM.SHAPE.C.SIZE-1) of Q_ELEM_TYPE;
    signal    q_element       :  Q_ELEM_VECTOR;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    q_data          :  std_logic_vector(Q_PARAM.DATA.SIZE-1 downto 0);
    signal    q_valid         :  std_logic;
    signal    q_ready         :  std_logic;
    signal    output_data     :  std_logic_vector(Q_PARAM.DATA.SIZE-1 downto 0);
begin
    -------------------------------------------------------------------------------
    -- i_element : 入力パイプラインイメージデータを要素ごとの配列に変換
    -- i_c_valid : 入力パイプラインイメージデータのチャネル有効信号
    -------------------------------------------------------------------------------
    process (I_DATA) begin
        for y in 0 to I_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to I_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to I_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to I_PARAM.SHAPE.C.SIZE-1 loop
            i_element(y,x,d,c) <= GET_ELEMENT_FROM_IMAGE_STREAM_DATA(I_PARAM, c, d, x, y, I_DATA);
        end loop;
        end loop;
        end loop;
        end loop;
        i_c_atrb <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(I_PARAM, I_DATA);
    end process;
    -------------------------------------------------------------------------------
    -- k_element : 入力パイプライン重みデータを要素ごとの配列に変換
    -- k_c_valid : 入力パイプライン重みデータのチャネル有効信号
    -------------------------------------------------------------------------------
    process (K_DATA) begin
        for y in 0 to K_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to K_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to K_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to K_PARAM.SHAPE.C.SIZE-1 loop
            k_element(y,x,d,c) <= GET_ELEMENT_FROM_IMAGE_STREAM_DATA(K_PARAM, c, d, x, y, K_DATA);
        end loop;
        end loop;
        end loop;
        end loop;
        k_c_atrb <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(K_PARAM, K_DATA);
    end process;
    -------------------------------------------------------------------------------
    -- o_element : 乗算結果
    -------------------------------------------------------------------------------
    process(i_element, i_c_atrb, k_element, k_c_atrb)
        variable i_data  :  std_logic_vector(QCONV_PARAM.NBITS_IN_DATA  downto 0);
        variable k_data  :  std_logic_vector(QCONV_PARAM.NBITS_K_DATA-1 downto 0);
        variable o_data  :  std_logic_vector(O_PARAM.ELEM_BITS       -1 downto 0);
    begin
        for y in 0 to Q_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to Q_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to Q_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to Q_PARAM.SHAPE.C.SIZE-1 loop
            for i in 0 to QCONV_PARAM.NBITS_PER_WORD-1 loop
                if (i_c_atrb(c).VALID = TRUE and CHECK_K_VALID /= 0 and k_c_atrb(c).VALID = TRUE) or
                   (i_c_atrb(c).VALID = TRUE and CHECK_K_VALID  = 0                             ) then
                    i_data := "0" & i_element(y,x,d,c)((i+1)*QCONV_PARAM.NBITS_IN_DATA-1 downto i*QCONV_PARAM.NBITS_IN_DATA);
                    k_data :=       k_element(y,x,d,c)((i+1)*QCONV_PARAM.NBITS_K_DATA -1 downto i*QCONV_PARAM.NBITS_K_DATA );
                    if (k_data(0) = '1') then
                        o_data := std_logic_vector(resize( signed(i_data), O_PARAM.ELEM_BITS));
                    else
                        o_data := std_logic_vector(resize(-signed(i_data), O_PARAM.ELEM_BITS));
                    end if;
                else
                        o_data := std_logic_vector(to_signed(0, O_PARAM.ELEM_BITS));
                end if;                         
                q_element(y,x,d,c)((i+1)*O_PARAM.ELEM_BITS-1 downto i*O_PARAM.ELEM_BITS) <= o_data;
            end loop;
        end loop;
        end loop;
        end loop;
        end loop;
    end process;
    -------------------------------------------------------------------------------
    -- q_data    : パイプラインレジスタに入力するデータ
    -------------------------------------------------------------------------------
    process (q_element, I_DATA)
        variable data :  std_logic_vector(Q_PARAM.DATA.SIZE-1 downto 0);
    begin
        for y in 0 to Q_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to Q_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to Q_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to Q_PARAM.SHAPE.C.SIZE-1 loop
            SET_ELEMENT_TO_IMAGE_STREAM_DATA(Q_PARAM, c, d, x, y, q_element(y,x,d,c), data);
        end loop;        
        end loop;        
        end loop;        
        end loop;
        if (Q_PARAM.DATA.ATRB_FIELD.SIZE > 0) then
            data(Q_PARAM.DATA.ATRB_FIELD.HI downto Q_PARAM.DATA.ATRB_FIELD.LO) := I_DATA(I_PARAM.DATA.ATRB_FIELD.HI downto I_PARAM.DATA.ATRB_FIELD.LO);
        end if;
        if (Q_PARAM.DATA.INFO_FIELD.SIZE > 0) then
            data(Q_PARAM.DATA.INFO_FIELD.HI downto Q_PARAM.DATA.INFO_FIELD.LO) := I_DATA(I_PARAM.DATA.INFO_FIELD.HI downto I_PARAM.DATA.INFO_FIELD.LO);
        end if;
        q_data <= data;
    end process;
    -------------------------------------------------------------------------------
    -- q_valid   : 
    -------------------------------------------------------------------------------
    q_valid <= '1' when (I_VALID = '1' and K_VALID = '1') else '0';
    I_READY <= '1' when (q_valid = '1' and q_ready = '1') else '0';
    K_READY <= '1' when (q_valid = '1' and q_ready = '1') else '0';
    -------------------------------------------------------------------------------
    -- パイプラインレジスタ
    -------------------------------------------------------------------------------
    QUEUE: PIPELINE_REGISTER                   -- 
        generic map (                          -- 
            QUEUE_SIZE  => QUEUE_SIZE        , --
            WORD_BITS   => Q_PARAM.DATA.SIZE   -- 
        )                                      -- 
        port map (                             -- 
            CLK         => CLK               , -- In  :
            RST         => RST               , -- In  :
            CLR         => CLR               , -- In  :
            I_WORD      => q_data            , -- In  :
            I_VAL       => q_valid           , -- In  :
            I_RDY       => q_ready           , -- Out :
            Q_WORD      => output_data       , -- Out :
            Q_VAL       => O_VALID           , -- Out :
            Q_RDY       => O_READY           , -- In  :
            BUSY        => open                -- Out :
        );                                     -- 
    -------------------------------------------------------------------------------
    -- O_DATA
    -------------------------------------------------------------------------------
    process (output_data)
        variable    data            :  std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        variable    q_elem          :  Q_ELEM_TYPE;
        variable    q_c_atrb_vector :  IMAGE_STREAM_ATRB_VECTOR(0 to Q_PARAM.SHAPE.C.SIZE-1);
        variable    o_c_atrb_vector :  IMAGE_STREAM_ATRB_VECTOR(0 to O_PARAM.SHAPE.C.SIZE-1);
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        for y in 0 to Q_PARAM.SHAPE.Y.SIZE-1 loop
        for x in 0 to Q_PARAM.SHAPE.X.SIZE-1 loop
        for d in 0 to Q_PARAM.SHAPE.D.SIZE-1 loop
        for c in 0 to Q_PARAM.SHAPE.C.SIZE-1 loop
            q_elem := GET_ELEMENT_FROM_IMAGE_STREAM_DATA(Q_PARAM, c, d, x, y, output_data);
            for i in 0 to QCONV_PARAM.NBITS_PER_WORD-1 loop
                SET_ELEMENT_TO_IMAGE_STREAM_DATA(
                    PARAM    => O_PARAM,
                    C        => c*QCONV_PARAM.NBITS_PER_WORD+i,
                    D        => d,
                    X        => x,
                    Y        => y,
                    ELEMENT  => q_elem((i+1)*O_PARAM.ELEM_BITS-1 downto i*O_PARAM.ELEM_BITS),
                    DATA     => data
                );
            end loop;
        end loop;       
        end loop;       
        end loop;       
        end loop;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        q_c_atrb_vector := GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(Q_PARAM, output_data);
        for c in 0 to Q_PARAM.SHAPE.C.SIZE-1 loop
            for i in 0 to QCONV_PARAM.NBITS_PER_WORD-1 loop
                o_c_atrb_vector(c*QCONV_PARAM.NBITS_PER_WORD+i).VALID := q_c_atrb_vector(c).VALID;
                o_c_atrb_vector(c*QCONV_PARAM.NBITS_PER_WORD+i).START := FALSE;
                o_c_atrb_vector(c*QCONV_PARAM.NBITS_PER_WORD+i).LAST  := FALSE;
            end loop;
        end loop;
        o_c_atrb_vector(o_c_atrb_vector'low ).START := q_c_atrb_vector(q_c_atrb_vector'low ).START;
        o_c_atrb_vector(o_c_atrb_vector'high).LAST  := q_c_atrb_vector(q_c_atrb_vector'high).LAST;
        SET_ATRB_C_VECTOR_TO_IMAGE_STREAM_DATA(O_PARAM, o_c_atrb_vector, data);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        SET_ATRB_D_VECTOR_TO_IMAGE_STREAM_DATA(O_PARAM, GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(Q_PARAM, output_data), data);
        SET_ATRB_X_VECTOR_TO_IMAGE_STREAM_DATA(O_PARAM, GET_ATRB_X_VECTOR_FROM_IMAGE_STREAM_DATA(Q_PARAM, output_data), data);
        SET_ATRB_Y_VECTOR_TO_IMAGE_STREAM_DATA(O_PARAM, GET_ATRB_Y_VECTOR_FROM_IMAGE_STREAM_DATA(Q_PARAM, output_data), data);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        O_DATA <= data;
    end process;
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_in_data_buffer.vhd
--!     @brief   Quantized Convolution (strip) In Data Buffer Module
--!     @version 0.1.0
--!     @date    2019/4/5
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_IN_DATA_BUFFER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        O_PARAM         : --! @brief OUTPUT STREAM PARAMETER :
                          --! 出力側の IMAGE STREAM のパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                              ELEM_BITS => 64,
                              C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1*3*3*32),
                              D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                          );
        I_SHAPE         : --! @brief INPUT  SHAPE :
                          --! 入力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        O_SHAPE         : --! @brief OUTPUT SHAPE :
                          --! 出力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        ELEMENT_SIZE    : --! @brief ELEMENT SIZE :
                          --! 列方向の要素数を指定する.
                          integer := 256;
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 1;
        ID              : --! @brief SDPRAM IDENTIFIER :
                          --! どのモジュールで使われているかを示す識別番号.
                          integer := 0 
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : --! @brief INPUT C CHANNEL SIZE :
                          in  integer range 0 to I_SHAPE.C.MAX_SIZE := I_SHAPE.C.SIZE;
        IN_W            : --! @brief INPUT IMAGE WIDTH :
                          in  integer range 0 to I_SHAPE.X.MAX_SIZE := I_SHAPE.X.SIZE;
        IN_H            : --! @brief INPUT IMAGE HEIGHT :
                          in  integer range 0 to I_SHAPE.Y.MAX_SIZE := I_SHAPE.Y.SIZE;
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  integer range 0 to O_SHAPE.C.MAX_SIZE := O_SHAPE.C.SIZE;
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  integer range 0 to O_SHAPE.X.MAX_SIZE := O_SHAPE.X.SIZE;
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  integer range 0 to O_SHAPE.Y.MAX_SIZE := O_SHAPE.Y.SIZE;
        K3x3            : --! @brief KERNEL SIZE :
                          --! * Kernel が 3x3 の場合は'1'.
                          --! * Kernel が 1x1 の場合は'0'.
                          in  std_logic;
        LEFT_PAD_SIZE   : --! @brief IMAGE WIDTH START PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        RIGHT_PAD_SIZE  : --! @brief IMAGE WIDTH LAST  PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        TOP_PAD_SIZE    : --! @brief IMAGE HEIGHT START PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        BOTTOM_PAD_SIZE : --! @brief IMAGE HEIGHT LAST  PAD SIZE :
                          in  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE := 0;
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT IN_DATA :
                          --! IN_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        I_VALID         : --! @brief INPUT IN_DATA VALID :
                          --! IN_DATA 入力有効信号.
                          in  std_logic;
        I_READY         : --! @brief INPUT IN_DATA READY :
                          --! IN_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT IMAGE STREAM DATA :
                          --! ストリームデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT IMAGE STREAM DATA VALID :
                          --! 出力ストリームデータ有効信号.
                          out std_logic;
        O_READY         : --! @brief OUTPUT IMAGE STREAM DATA READY :
                          --! 出力ストリームデータレディ信号.
                          in  std_logic
    );
end QCONV_STRIP_IN_DATA_BUFFER;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
library PIPEWORK;
use     PIPEWORK.COMPONENTS.SDPRAM;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_STREAM_GENERATOR;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_STREAM_CHANNEL_REDUCER;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_STREAM_BUFFER_INTAKE;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_STREAM_BUFFER_OUTLET;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_STREAM_GENERATOR_WITH_PADDING;
use     PIPEWORK.CONVOLUTION_TYPES.all;
architecture RTL of QCONV_STRIP_IN_DATA_BUFFER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  WORD_BITS             :  integer := QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD;
    constant  PAD_DATA              :  std_logic_vector(WORD_BITS-1 downto 0) := (others => '0');
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  BUF_BANK_SIZE         :  integer := 4;
    constant  BUF_LINE_SIZE         :  integer := 4;
    -------------------------------------------------------------------------------
    -- BUF_WIDTH : メモリのビット幅を２のべき乗値で示す
    -------------------------------------------------------------------------------
    function  CALC_BUF_WIDTH    return integer is
        variable width              :  integer;
    begin
        width := 0;
        while (2**width < (IN_C_UNROLL * WORD_BITS)) loop
            width := width + 1;
        end loop;
        return width;
    end function;
    constant  BUF_WIDTH             :  integer := CALC_BUF_WIDTH;
    -------------------------------------------------------------------------------
    -- BUF_DEPTH: メモリバンク１つあたりの深さ(ビット単位)を２のべき乗値で示す
    -------------------------------------------------------------------------------
    function  CALC_BUF_DEPTH    return integer is
        variable size               :  integer;
        variable depth              :  integer;
    begin
        size  := ELEMENT_SIZE*WORD_BITS;
        size  := (size + BUF_BANK_SIZE - 1)/BUF_BANK_SIZE;
        depth := 0;
        while (2**depth < size) loop
            depth := depth + 1;
        end loop;
        return depth;
    end function;
    constant  BUF_DEPTH             :  integer := CALC_BUF_DEPTH;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  BUF_DATA_BITS         :  integer := 2**BUF_WIDTH;
    constant  BUF_ADDR_BITS         :  integer := BUF_DEPTH - BUF_WIDTH;
    constant  BUF_WENA_BITS         :  integer := 1;
    signal    conv3x3_buf_wdata     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv3x3_buf_waddr     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    signal    conv3x3_buf_we        :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_WENA_BITS-1 downto 0);
    signal    conv3x3_buf_rdata     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv3x3_buf_raddr     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    signal    conv1x1_buf_wdata     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv1x1_buf_waddr     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    signal    conv1x1_buf_we        :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_WENA_BITS-1 downto 0);
    signal    conv1x1_buf_rdata     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv1x1_buf_raddr     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  INTAKE_STREAM_PARAM   :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                           ELEM_BITS => WORD_BITS,
                                           C         => 1,
                                           X         => 1,
                                           Y         => 1
                                       );
    signal    intake_start          :  std_logic;
    signal    intake_busy           :  std_logic;
    signal    intake_done           :  std_logic;
    signal    conv3x3_intake_data   :  std_logic_vector(INTAKE_STREAM_PARAM.DATA.SIZE-1 downto 0);
    signal    conv3x3_intake_valid  :  std_logic;
    signal    conv3x3_intake_ready  :  std_logic;
    signal    conv1x1_intake_data   :  std_logic_vector(INTAKE_STREAM_PARAM.DATA.SIZE-1 downto 0);
    signal    conv1x1_intake_valid  :  std_logic;
    signal    conv1x1_intake_ready  :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  OUTLET_STREAM_PARAM   :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                           ELEM_BITS => WORD_BITS,
                                           C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(3*3*IN_C_UNROLL, TRUE, TRUE),
                                           D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(   OUT_C_UNROLL, TRUE, TRUE),
                                           X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1              , TRUE, TRUE),
                                           Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1              , TRUE, TRUE)
                                       );
    signal    conv3x3_outlet_data   :  std_logic_vector(OUTLET_STREAM_PARAM.DATA.SIZE-1 downto 0);
    signal    conv3x3_outlet_valid  :  std_logic;
    signal    conv3x3_outlet_ready  :  std_logic;
    signal    conv3x3_outlet_atrb_c :  IMAGE_STREAM_ATRB_VECTOR(0 to OUTLET_STREAM_PARAM.SHAPE.C.SIZE-1);
    signal    conv3x3_outlet_atrb_d :  IMAGE_STREAM_ATRB_VECTOR(0 to OUTLET_STREAM_PARAM.SHAPE.D.SIZE-1);
    signal    conv3x3_outlet_atrb_x :  IMAGE_STREAM_ATRB_VECTOR(0 to OUTLET_STREAM_PARAM.SHAPE.X.SIZE-1);
    signal    conv3x3_outlet_atrb_y :  IMAGE_STREAM_ATRB_VECTOR(0 to OUTLET_STREAM_PARAM.SHAPE.Y.SIZE-1);
    signal    conv1x1_outlet_data   :  std_logic_vector(OUTLET_STREAM_PARAM.DATA.SIZE-1 downto 0);
    signal    conv1x1_outlet_valid  :  std_logic;
    signal    conv1x1_outlet_ready  :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      STATE_TYPE            is (IDLE_STATE, CONV3x3_STATE, CONV1x1_STATE, RES_STATE);
    signal    state                 :  STATE_TYPE;
    signal    conv3x3_start         :  std_logic;
    signal    conv3x3_busy          :  std_logic;
    signal    conv3x3_done          :  std_logic;
    signal    conv1x1_start         :  std_logic;
    signal    conv1x1_busy          :  std_logic;
    signal    conv1x1_done          :  std_logic;
begin
    -------------------------------------------------------------------------------
    -- メインシーケンサ
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                state <= IDLE_STATE;
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                state <= IDLE_STATE;
            else
                case state is
                    when IDLE_STATE =>
                        if    (REQ_VALID = '1' and K3x3  = '1') then
                            state <= CONV3x3_STATE;
                        elsif (REQ_VALID = '1' and K3x3 /= '1') then
                            state <= CONV1x1_STATE;
                        else
                            state <= IDLE_STATE;
                        end if;
                    when CONV3x3_STATE =>
                        if (intake_busy = '0' and (conv3x3_busy = '0' or conv3x3_done = '1')) then
                            state <= RES_STATE;
                        else
                            state <= CONV3x3_STATE;
                        end if;
                    when CONV1x1_STATE =>
                        if (intake_busy = '0' and (conv1x1_busy = '0' or conv1x1_done = '1')) then
                            state <= RES_STATE;
                        else
                            state <= CONV1x1_STATE;
                        end if;
                    when RES_STATE =>
                        if (RES_READY = '1') then
                            state <= IDLE_STATE;
                        else
                            state <= RES_STATE;
                        end if;
                    when others =>
                            state <= IDLE_STATE;
                end case;
            end if;
        end if;
    end process;
    REQ_READY     <= '1' when (state = IDLE_STATE) else '0';
    RES_VALID     <= '1' when (state = RES_STATE ) else '0';
    intake_start  <= '1' when (state = IDLE_STATE and REQ_VALID = '1') else '0';
    conv3x3_start <= '1' when (state = IDLE_STATE and REQ_VALID = '1' and K3x3  = '1') else '0';
    conv1x1_start <= '1' when (state = IDLE_STATE and REQ_VALID = '1' and K3x3 /= '1') else '0';
    -------------------------------------------------------------------------------
    -- 入力データを内部データ(INTAKE_STREAM)形式に変換する.
    -------------------------------------------------------------------------------
    INTAKE: block
        signal    swapped_data  :  std_logic_vector(QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        signal    o_data        :  std_logic_vector(INTAKE_STREAM_PARAM.DATA.SIZE-1 downto 0);
        signal    o_valid       :  std_logic;
        signal    o_ready       :  std_logic;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process(I_DATA) begin
            for word_pos in 0 to QCONV_PARAM.NBITS_PER_WORD-1 loop
            for data_pos in 0 to QCONV_PARAM.NBITS_IN_DATA -1 loop
                swapped_data(word_pos*QCONV_PARAM.NBITS_IN_DATA + data_pos) <= I_DATA(data_pos*QCONV_PARAM.NBITS_PER_WORD + word_pos);
            end loop;
            end loop;
        end process;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        PADDING: IMAGE_STREAM_GENERATOR_WITH_PADDING     -- 
            generic map (                                -- 
                O_PARAM         => INTAKE_STREAM_PARAM , -- 
                O_SHAPE         => I_SHAPE             , -- 
                I_DATA_BITS     => WORD_BITS           , --
                MAX_PAD_SIZE    => QCONV_PARAM.MAX_PAD_SIZE  -- 
            )                                            -- 
            port map (                                   -- 
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
                START           => intake_start        , -- In  :
                BUSY            => intake_busy         , -- Out :
                DONE            => intake_done         , -- Out :
                C_SIZE          => IN_C_BY_WORD        , -- In  :
                X_SIZE          => IN_W                , -- In  :
                Y_SIZE          => IN_H                , -- In  :
                LEFT_PAD_SIZE   => LEFT_PAD_SIZE       , -- In  :
                RIGHT_PAD_SIZE  => RIGHT_PAD_SIZE      , -- In  :
                TOP_PAD_SIZE    => TOP_PAD_SIZE        , -- In  :
                BOTTOM_PAD_SIZE => BOTTOM_PAD_SIZE     , -- In  :
                PAD_DATA        => PAD_DATA            , -- In  :
                I_DATA          => swapped_data        , -- In  :
                I_VALID         => I_VALID             , -- In  :
                I_READY         => I_READY             , -- Out :
                O_DATA          => o_data              , -- Out :
                O_VALID         => o_valid             , -- Out :
                O_READY         => o_ready               -- In  :
            );
        conv3x3_intake_data  <= o_data;
        conv1x1_intake_data  <= o_data;
        conv3x3_intake_valid <= '1' when (state = CONV3x3_STATE and o_valid  = '1') else '0';
        conv1x1_intake_valid <= '1' when (state = CONV1x1_STATE and o_valid  = '1') else '0';
        o_ready  <= '1' when (state = CONV3x3_STATE and conv3x3_intake_ready = '1') or
                             (state = CONV1x1_STATE and conv1x1_intake_ready = '1') else '0';
    end block;
    -------------------------------------------------------------------------------
    -- 3x3 用のバッファ制御
    -------------------------------------------------------------------------------
    CONV3x3: block
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  KERNEL_SIZE   :  CONVOLUTION_KERNEL_SIZE_TYPE   := CONVOLUTION_KERNEL_SIZE_3x3;
        constant  STRIDE        :  IMAGE_STREAM_STRIDE_PARAM_TYPE := NEW_IMAGE_STREAM_STRIDE_PARAM(1,1);
        constant  BANK_SIZE     :  integer := BUF_BANK_SIZE;
        constant  LINE_SIZE     :  integer := BUF_LINE_SIZE;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  I_CHAN_PARAM  :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => WORD_BITS,
                                       C         => IN_C_UNROLL,
                                       X         => 1,
                                       Y         => 1
                                   );
        signal    i_chan_data   :  std_logic_vector(I_CHAN_PARAM.DATA.SIZE-1 downto 0);
        signal    i_chan_valid  :  std_logic;
        signal    i_chan_ready  :  std_logic;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  O_CHAN_PARAM  :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => WORD_BITS,
                                       C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(IN_C_UNROLL                       , TRUE , TRUE ),
                                       D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(OUT_C_UNROLL                      , FALSE, TRUE ),
                                       X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.X.LO, KERNEL_SIZE.X.HI, TRUE , TRUE ),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.Y.LO, KERNEL_SIZE.Y.HI, TRUE , TRUE ),
                                       STRIDE    => STRIDE
                                   );
        constant  O_CHAN_SHAPE  :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                       ELEM_BITS => WORD_BITS,
                                       C         => NEW_IMAGE_SHAPE_SIDE_AUTO(ELEMENT_SIZE),
                                       D         => O_SHAPE.C,
                                       X         => NEW_IMAGE_SHAPE_SIDE_AUTO(ELEMENT_SIZE),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_AUTO(ELEMENT_SIZE)
                                   );
        signal    o_chan_data   :  std_logic_vector(O_CHAN_PARAM.DATA.SIZE-1 downto 0);
        signal    o_chan_valid  :  std_logic;
        signal    o_chan_ready  :  std_logic;
        signal    o_line_last   :  std_logic;
        signal    o_line_feed   :  std_logic;
        signal    o_line_return :  std_logic;
        signal    o_frame_last  :  std_logic;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    line_valid    :  std_logic_vector(        LINE_SIZE-1 downto 0);
        signal    line_atrb     :  IMAGE_STREAM_ATRB_VECTOR(LINE_SIZE-1 downto 0);
        signal    line_feed     :  std_logic_vector(        LINE_SIZE-1 downto 0);
        signal    line_return   :  std_logic_vector(        LINE_SIZE-1 downto 0);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    x_size        :  integer range 0 to ELEMENT_SIZE;
        signal    c_size        :  integer range 0 to ELEMENT_SIZE;
        signal    c_offset      :  integer range 0 to 2**BUF_ADDR_BITS;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    buf_wdata     :  std_logic_vector(BANK_SIZE*LINE_SIZE*BUF_DATA_BITS-1 downto 0);
        signal    buf_waddr     :  std_logic_vector(BANK_SIZE*LINE_SIZE*BUF_ADDR_BITS-1 downto 0);
        signal    buf_we        :  std_logic_vector(BANK_SIZE*LINE_SIZE              -1 downto 0);
        signal    buf_raddr     :  std_logic_vector(BANK_SIZE*LINE_SIZE*BUF_ADDR_BITS-1 downto 0);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    o_chan_atrb_c :  IMAGE_STREAM_ATRB_VECTOR(0 to O_CHAN_PARAM.SHAPE.C.SIZE-1);
        signal    o_chan_atrb_d :  IMAGE_STREAM_ATRB_VECTOR(0 to O_CHAN_PARAM.SHAPE.D.SIZE-1);
        signal    o_chan_atrb_x :  IMAGE_STREAM_ATRB_VECTOR(0 to O_CHAN_PARAM.SHAPE.X.SIZE-1);
        signal    o_chan_atrb_y :  IMAGE_STREAM_ATRB_VECTOR(0 to O_CHAN_PARAM.SHAPE.Y.SIZE-1);
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK,RST) begin
            if (RST = '1') then
                    conv3x3_busy <= '0';
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    conv3x3_busy <= '0';
                elsif (conv3x3_start = '1') then
                    conv3x3_busy <= '1';
                elsif (conv3x3_busy = '1' and o_frame_last  = '1') then
                    conv3x3_busy <= '0';
                end if;
            end if;
        end process;
        conv3x3_done <= '1' when (conv3x3_busy = '1' and o_frame_last  = '1') else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        I_CHAN: IMAGE_STREAM_CHANNEL_REDUCER             -- 
            generic map (                                -- 
                I_PARAM         => INTAKE_STREAM_PARAM , -- 
                O_PARAM         => I_CHAN_PARAM          -- 
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 入力側 Stream I/F
            -----------------------------------------------------------------------
                I_DATA          => conv3x3_intake_data , -- In  :
                I_VALID         => conv3x3_intake_valid, -- In  :
                I_READY         => conv3x3_intake_ready, -- Out :
            -----------------------------------------------------------------------
            -- 出力側 Stream I/F
            -----------------------------------------------------------------------
                O_DATA          => i_chan_data         , -- Out :
                O_VALID         => i_chan_valid        , -- Out :
                O_READY         => i_chan_ready          -- In  :
            );                                           -- 
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        BUF_INTAKE: IMAGE_STREAM_BUFFER_INTAKE           -- 
            generic map (                                -- 
                I_PARAM         => I_CHAN_PARAM        , -- 
                I_SHAPE         => I_SHAPE             , --
                ELEMENT_SIZE    => ELEMENT_SIZE        , --
                BANK_SIZE       => BANK_SIZE           , --
                LINE_SIZE       => LINE_SIZE           , --
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS       , -- 
                LINE_QUEUE      => 1                     --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 入力側 I/F
            -----------------------------------------------------------------------
                I_DATA          => i_chan_data         , -- In  :
                I_VALID         => i_chan_valid        , -- In  :
                I_READY         => i_chan_ready        , -- Out :
            -----------------------------------------------------------------------
            -- 出力側 I/F
            -----------------------------------------------------------------------
                O_LINE_VALID    => line_valid          , -- Out :
                O_X_SIZE        => x_size              , -- Out :
                O_C_SIZE        => c_size              , -- Out :
                O_C_OFFSET      => c_offset            , -- Out :
                O_LINE_ATRB     => line_atrb           , -- Out :
                O_LINE_FEED     => line_feed           , -- In  :
                O_LINE_RETURN   => line_return         , -- In  :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => buf_wdata           , -- Out :
                BUF_ADDR        => buf_waddr           , -- Out :
                BUF_WE          => buf_we                -- Out :
            );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        BUF_OUTLET: IMAGE_STREAM_BUFFER_OUTLET           -- 
            generic map (                                -- 
                O_PARAM         => O_CHAN_PARAM        , --
                O_SHAPE         => O_CHAN_SHAPE        , --
                ELEMENT_SIZE    => ELEMENT_SIZE        , --
                BANK_SIZE       => BANK_SIZE           , --
                LINE_SIZE       => LINE_SIZE           , --
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS       , --
                BANK_QUEUE      => 2                   , --
                LINE_QUEUE      => 1                     --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 各種サイズ
            -----------------------------------------------------------------------
                X_SIZE          => x_size              , -- In  :
                D_SIZE          => OUT_C               , -- In  :
                C_SIZE          => c_size              , -- In  :
                C_OFFSET        => c_offset            , -- In  :
            -----------------------------------------------------------------------
            -- 入力側 I/F
            -----------------------------------------------------------------------
                I_LINE_VALID    => line_valid          , -- In  :
                I_LINE_ATRB     => line_atrb           , -- In  :
                I_LINE_FEED     => line_feed           , -- Out :
                I_LINE_RETURN   => line_return         , -- Out :
            -----------------------------------------------------------------------
            -- 出力側 I/F
            -----------------------------------------------------------------------
                O_DATA          => o_chan_data         , -- Out :
                O_VALID         => o_chan_valid        , -- Out :
                O_READY         => o_chan_ready        , -- In  :
                O_LAST          => o_frame_last        , -- In  :
                O_FEED          => o_line_feed         , -- In  :
                O_RETURN        => o_line_return       , -- In  :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => conv3x3_buf_rdata   , -- In  :
                BUF_ADDR        => buf_raddr             -- Out :
            );
        ---------------------------------------------------------------------------
        -- 
        ---------------------------------------------------------------------------
        o_line_last  <= '1' when (IMAGE_STREAM_DATA_IS_LAST_C(O_CHAN_PARAM, o_chan_data)) and
                                 (IMAGE_STREAM_DATA_IS_LAST_D(O_CHAN_PARAM, o_chan_data)) and
                                 (IMAGE_STREAM_DATA_IS_LAST_X(O_CHAN_PARAM, o_chan_data)) and
                                 (o_chan_valid = '1' and o_chan_ready = '1'             ) else '0';
        o_frame_last <= '1' when (o_line_last  = '1') and
                                 (IMAGE_STREAM_DATA_IS_LAST_Y(O_CHAN_PARAM, o_chan_data)) else '0';
        o_line_feed  <= '1' when (o_line_last  = '1') else '0';
        o_line_return<= '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv3x3_buf_wdata <= buf_wdata when (conv3x3_busy = '1') else (others => '0');
        conv3x3_buf_waddr <= buf_waddr when (conv3x3_busy = '1') else (others => '0');
        conv3x3_buf_we    <= buf_we    when (conv3x3_busy = '1') else (others => '0');
        conv3x3_buf_raddr <= buf_raddr when (conv3x3_busy = '1') else (others => '0');
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv3x3_outlet_data <= CONVOLUTION_PIPELINE_FROM_IMAGE_STREAM(
                                    PIPELINE_PARAM => OUTLET_STREAM_PARAM,
                                    STREAM_PARAM   => O_CHAN_PARAM       ,
                                    KERNEL_SIZE    => KERNEL_SIZE        ,
                                    STRIDE         => STRIDE             ,
                                    STREAM_DATA    => o_chan_data
                                );
        conv3x3_outlet_valid <= o_chan_valid;
        o_chan_ready <= conv3x3_outlet_ready;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        o_chan_atrb_c <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(O_CHAN_PARAM, o_chan_data);
        o_chan_atrb_d <= GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(O_CHAN_PARAM, o_chan_data);
        o_chan_atrb_x <= GET_ATRB_X_VECTOR_FROM_IMAGE_STREAM_DATA(O_CHAN_PARAM, o_chan_data);
        o_chan_atrb_y <= GET_ATRB_Y_VECTOR_FROM_IMAGE_STREAM_DATA(O_CHAN_PARAM, o_chan_data);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv3x3_outlet_atrb_c <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(OUTLET_STREAM_PARAM, conv3x3_outlet_data);
        conv3x3_outlet_atrb_d <= GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(OUTLET_STREAM_PARAM, conv3x3_outlet_data);
        conv3x3_outlet_atrb_x <= GET_ATRB_X_VECTOR_FROM_IMAGE_STREAM_DATA(OUTLET_STREAM_PARAM, conv3x3_outlet_data);
        conv3x3_outlet_atrb_y <= GET_ATRB_Y_VECTOR_FROM_IMAGE_STREAM_DATA(OUTLET_STREAM_PARAM, conv3x3_outlet_data);
    end block;
    -------------------------------------------------------------------------------
    -- 1x1 用のバッファ制御
    -------------------------------------------------------------------------------
    CONV1x1: block
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  KERNEL_SIZE   :  CONVOLUTION_KERNEL_SIZE_TYPE   := CONVOLUTION_KERNEL_SIZE_1x1;
        constant  STRIDE        :  IMAGE_STREAM_STRIDE_PARAM_TYPE := NEW_IMAGE_STREAM_STRIDE_PARAM(1,1);
        constant  BANK_SIZE     :  integer := 1;
        constant  LINE_SIZE     :  integer := 2;
        constant  C_WORDS       :  integer := 8;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  I_CHAN_PARAM  :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => WORD_BITS,
                                       C         => IN_C_UNROLL*C_WORDS,
                                       X         => 1,
                                       Y         => 1
                                   );
        signal    i_chan_data   :  std_logic_vector(I_CHAN_PARAM.DATA.SIZE-1 downto 0);
        signal    i_chan_valid  :  std_logic;
        signal    i_chan_ready  :  std_logic;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  O_CHAN_PARAM  :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => WORD_BITS,
                                       C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(IN_C_UNROLL*C_WORDS               , TRUE , TRUE ),
                                       D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(OUT_C_UNROLL                      , FALSE, TRUE ),
                                       X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.X.LO, KERNEL_SIZE.X.HI, TRUE , TRUE ),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.Y.LO, KERNEL_SIZE.Y.HI, TRUE , TRUE ),
                                       STRIDE    => STRIDE
                                   );
        constant  O_CHAN_SHAPE  :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                       ELEM_BITS => WORD_BITS,
                                       C         => NEW_IMAGE_SHAPE_SIDE_AUTO(ELEMENT_SIZE),
                                       D         => O_SHAPE.C,
                                       X         => NEW_IMAGE_SHAPE_SIDE_AUTO(ELEMENT_SIZE),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_AUTO(ELEMENT_SIZE)
                                   );
        signal    o_chan_data   :  std_logic_vector(O_CHAN_PARAM.DATA.SIZE-1 downto 0);
        signal    o_chan_valid  :  std_logic;
        signal    o_chan_ready  :  std_logic;
        signal    o_line_last   :  std_logic;
        signal    o_line_feed   :  std_logic;
        signal    o_line_return :  std_logic;
        signal    o_frame_last  :  std_logic;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    line_valid    :  std_logic_vector(        LINE_SIZE-1 downto 0);
        signal    line_atrb     :  IMAGE_STREAM_ATRB_VECTOR(LINE_SIZE-1 downto 0);
        signal    line_feed     :  std_logic_vector(        LINE_SIZE-1 downto 0);
        signal    line_return   :  std_logic_vector(        LINE_SIZE-1 downto 0);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    x_size        :  integer range 0 to ELEMENT_SIZE;
        signal    c_size        :  integer range 0 to ELEMENT_SIZE;
        signal    c_offset      :  integer range 0 to 2**BUF_ADDR_BITS;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        signal    buf_wdata     :  std_logic_vector(BUF_LINE_SIZE*BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
        signal    buf_waddr     :  std_logic_vector(LINE_SIZE*BUF_ADDR_BITS-1 downto 0);
        signal    buf_we        :  std_logic_vector(LINE_SIZE              -1 downto 0);
        signal    buf_raddr     :  std_logic_vector(LINE_SIZE*BUF_ADDR_BITS-1 downto 0);
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK,RST) begin
            if (RST = '1') then
                    conv1x1_busy <= '0';
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    conv1x1_busy <= '0';
                elsif (conv1x1_start = '1') then
                    conv1x1_busy <= '1';
                elsif (conv1x1_busy = '1' and o_frame_last  = '1') then
                    conv1x1_busy <= '0';
                end if;
            end if;
        end process;
        conv1x1_done <= '1' when (conv1x1_busy = '1' and o_frame_last  = '1') else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        I_CHAN: IMAGE_STREAM_CHANNEL_REDUCER             -- 
            generic map (                                -- 
                I_PARAM         => INTAKE_STREAM_PARAM , -- 
                O_PARAM         => I_CHAN_PARAM          -- 
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 入力側 Stream I/F
            -----------------------------------------------------------------------
                I_DATA          => conv1x1_intake_data , -- In  :
                I_VALID         => conv1x1_intake_valid, -- In  :
                I_READY         => conv1x1_intake_ready, -- Out :
            -----------------------------------------------------------------------
            -- 出力側 Stream I/F
            -----------------------------------------------------------------------
                O_DATA          => i_chan_data         , -- Out :
                O_VALID         => i_chan_valid        , -- Out :
                O_READY         => i_chan_ready          -- In  :
            );                                           -- 
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        BUF_INTAKE: IMAGE_STREAM_BUFFER_INTAKE           -- 
            generic map (                                -- 
                I_PARAM         => I_CHAN_PARAM        , -- 
                I_SHAPE         => I_SHAPE             , --
                ELEMENT_SIZE    => ELEMENT_SIZE        , --
                BANK_SIZE       => BANK_SIZE           , --
                LINE_SIZE       => LINE_SIZE           , --
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS*C_WORDS,--
                LINE_QUEUE      => 1                     --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 入力側 I/F
            -----------------------------------------------------------------------
                I_DATA          => i_chan_data         , -- In  :
                I_VALID         => i_chan_valid        , -- In  :
                I_READY         => i_chan_ready        , -- Out :
            -----------------------------------------------------------------------
            -- 出力側 I/F
            -----------------------------------------------------------------------
                O_LINE_VALID    => line_valid          , -- Out :
                O_X_SIZE        => x_size              , -- Out :
                O_C_SIZE        => c_size              , -- Out :
                O_C_OFFSET      => c_offset            , -- Out :
                O_LINE_ATRB     => line_atrb           , -- Out :
                O_LINE_FEED     => line_feed           , -- In  :
                O_LINE_RETURN   => line_return         , -- In  :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => buf_wdata           , -- Out :
                BUF_ADDR        => buf_waddr           , -- Out :
                BUF_WE          => buf_we                -- Out :
            );
        process (buf_waddr, conv1x1_busy) begin
            if (conv1x1_busy = '1') then
                for line  in 0 to LINE_SIZE-1 loop
                for c_pos in 0 to C_WORDS  -1 loop
                    conv1x1_buf_waddr((line*C_WORDS+c_pos+1)*BUF_ADDR_BITS-1 downto (line*C_WORDS+c_pos)*BUF_ADDR_BITS) <= buf_waddr((line+1)*BUF_ADDR_BITS-1 downto line*BUF_ADDR_BITS);
                end loop;
                end loop;
            else
                conv1x1_buf_waddr <= (others => '0');
            end if;
        end process;
        process (buf_we, conv1x1_busy) begin
            if (conv1x1_busy = '1') then
                for line  in 0 to LINE_SIZE-1 loop
                for c_pos in 0 to C_WORDS-1   loop
                    conv1x1_buf_we(line*C_WORDS+c_pos) <= buf_we(line);
                end loop;
                end loop;
            else
                conv1x1_buf_we <= (others => '0');
            end if;
        end process;
        conv1x1_buf_wdata <= buf_wdata when (conv1x1_busy = '1') else (others => '0');
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        BUF_OUTLET: IMAGE_STREAM_BUFFER_OUTLET           -- 
            generic map (                                -- 
                O_PARAM         => O_CHAN_PARAM        , --
                O_SHAPE         => O_CHAN_SHAPE        , --
                ELEMENT_SIZE    => ELEMENT_SIZE        , --
                BANK_SIZE       => BANK_SIZE           , --
                LINE_SIZE       => LINE_SIZE           , --
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS*C_WORDS,--
                BANK_QUEUE      => 2                   , --
                LINE_QUEUE      => 1                     --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 各種サイズ
            -----------------------------------------------------------------------
                X_SIZE          => x_size              , -- In  :
                D_SIZE          => OUT_C               , -- In  :
                C_SIZE          => c_size              , -- In  :
                C_OFFSET        => c_offset            , -- In  :
            -----------------------------------------------------------------------
            -- 入力側 I/F
            -----------------------------------------------------------------------
                I_LINE_VALID    => line_valid          , -- In  :
                I_LINE_ATRB     => line_atrb           , -- In  :
                I_LINE_FEED     => line_feed           , -- Out :
                I_LINE_RETURN   => line_return         , -- Out :
            -----------------------------------------------------------------------
            -- 出力側 I/F
            -----------------------------------------------------------------------
                O_DATA          => o_chan_data         , -- Out :
                O_VALID         => o_chan_valid        , -- Out :
                O_READY         => o_chan_ready        , -- In  :
                O_LAST          => o_frame_last        , -- In  :
                O_FEED          => o_line_feed         , -- In  :
                O_RETURN        => o_line_return       , -- In  :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => conv1x1_buf_rdata   , -- In  :
                BUF_ADDR        => buf_raddr             -- Out :
            );
        process (buf_raddr, conv1x1_busy) begin
            if (conv1x1_busy = '1') then
                for line  in 0 to LINE_SIZE-1 loop
                for c_pos in 0 to C_WORDS  -1 loop
                    conv1x1_buf_raddr((line*C_WORDS+c_pos+1)*BUF_ADDR_BITS-1 downto (line*C_WORDS+c_pos)*BUF_ADDR_BITS) <= buf_raddr((line+1)*BUF_ADDR_BITS-1 downto line*BUF_ADDR_BITS);
                end loop;
                end loop;
            else
                conv1x1_buf_raddr <= (others => '0');
            end if;
        end process;
        ---------------------------------------------------------------------------
        -- 
        ---------------------------------------------------------------------------
        o_line_last  <= '1' when (IMAGE_STREAM_DATA_IS_LAST_C(O_CHAN_PARAM, o_chan_data)) and
                                 (IMAGE_STREAM_DATA_IS_LAST_D(O_CHAN_PARAM, o_chan_data)) and
                                 (IMAGE_STREAM_DATA_IS_LAST_X(O_CHAN_PARAM, o_chan_data)) and
                                 (o_chan_valid = '1' and o_chan_ready = '1'             ) else '0';
        o_frame_last <= '1' when (o_line_last  = '1') and
                                 (IMAGE_STREAM_DATA_IS_LAST_Y(O_CHAN_PARAM, o_chan_data)) else '0';
        o_line_feed  <= '1' when (o_line_last  = '1') else '0';
        o_line_return<= '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv1x1_outlet_data <= CONVOLUTION_PIPELINE_FROM_IMAGE_STREAM(
                                    PIPELINE_PARAM => OUTLET_STREAM_PARAM,
                                    STREAM_PARAM   => O_CHAN_PARAM       ,
                                    KERNEL_SIZE    => KERNEL_SIZE        ,
                                    STRIDE         => STRIDE             ,
                                    STREAM_DATA    => o_chan_data
                                );
        conv1x1_outlet_valid <= o_chan_valid;
        o_chan_ready <= conv1x1_outlet_ready;
    end block;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    O_DATA  <= conv3x3_outlet_data when (conv3x3_busy = '1') else conv1x1_outlet_data;
    O_VALID <= '1' when (conv3x3_outlet_valid = '1' and conv3x3_busy = '1') or
                        (conv1x1_outlet_valid = '1' and conv1x1_busy = '1') else '0';
    conv3x3_outlet_ready <= '1' when (O_READY = '1' and conv3x3_busy = '1') else '0';
    conv1x1_outlet_ready <= '1' when (O_READY = '1' and conv1x1_busy = '1') else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    BUF_L:  for line in 0 to BUF_LINE_SIZE-1 generate
        B:  for bank in 0 to BUF_BANK_SIZE-1 generate
                constant  RAM_ID :  integer := ID + (line*BUF_BANK_SIZE)+bank;
                signal    wdata  :  std_logic_vector(BUF_DATA_BITS-1 downto 0);
                signal    waddr  :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
                signal    we     :  std_logic_vector(BUF_WENA_BITS-1 downto 0);
                signal    rdata  :  std_logic_vector(BUF_DATA_BITS-1 downto 0);
                signal    raddr  :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
            begin
            -----------------------------------------------------------------------
            --
            -----------------------------------------------------------------------
            wdata <= conv3x3_buf_wdata((line*BUF_BANK_SIZE+bank+1)*BUF_DATA_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_DATA_BITS) or
                     conv1x1_buf_wdata((line*BUF_BANK_SIZE+bank+1)*BUF_DATA_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_DATA_BITS);
            waddr <= conv3x3_buf_waddr((line*BUF_BANK_SIZE+bank+1)*BUF_ADDR_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_ADDR_BITS) or
                     conv1x1_buf_waddr((line*BUF_BANK_SIZE+bank+1)*BUF_ADDR_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_ADDR_BITS);
            we    <= conv3x3_buf_we   ((line*BUF_BANK_SIZE+bank+1)*BUF_WENA_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_WENA_BITS) or
                     conv1x1_buf_we   ((line*BUF_BANK_SIZE+bank+1)*BUF_WENA_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_WENA_BITS);
            raddr <= conv3x3_buf_raddr((line*BUF_BANK_SIZE+bank+1)*BUF_ADDR_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_ADDR_BITS) or
                     conv1x1_buf_raddr((line*BUF_BANK_SIZE+bank+1)*BUF_ADDR_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_ADDR_BITS);
            conv3x3_buf_rdata((line*BUF_BANK_SIZE+bank+1)*BUF_DATA_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_DATA_BITS) <= rdata;
            conv1x1_buf_rdata((line*BUF_BANK_SIZE+bank+1)*BUF_DATA_BITS-1 downto (line*BUF_BANK_SIZE+bank)*BUF_DATA_BITS) <= rdata;
            -----------------------------------------------------------------------
            --
            -----------------------------------------------------------------------
            RAM: SDPRAM                   -- 
                generic map (             -- 
                    DEPTH   => BUF_DEPTH, -- メモリの深さ(ビット単位)を2のべき乗値で指定する.
                    RWIDTH  => BUF_WIDTH, -- リードデータ(RDATA)の幅(ビット数)を2のべき乗値で指定する.
                    WWIDTH  => BUF_WIDTH, -- ライトデータ(WDATA)の幅(ビット数)を2のべき乗値で指定する.
                    WEBIT   => 0        , -- ライトイネーブル信号(WE)の幅(ビット数)を2のべき乗値で指定する.
                    ID      => RAM_ID     -- どのモジュールで使われているかを示す識別番号.
                )                         -- 
                port map (                -- 
                    WCLK    => CLK      , -- In  :
                    WE      => we       , -- In  : 
                    WADDR   => waddr    , -- In  : 
                    WDATA   => wdata    , -- In  : 
                    RCLK    => CLK      , -- In  :
                    RADDR   => raddr    , -- In  :
                    RDATA   => rdata      -- Out :
                );                        -- 
        end generate;
    end generate;
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_k_data_buffer.vhd
--!     @brief   Quantized Convolution (strip) Kernel Weight Data Buffer Module
--!     @version 0.1.0
--!     @date    2019/4/11
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_K_DATA_BUFFER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        O_PARAM         : --! @brief OUTPUT STREAM PARAMETER :
                          --! 出力側の IMAGE STREAM のパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                              ELEM_BITS => 32,
                              C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1*3*3),
                              D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                          );
        I_SHAPE         : --! @brief INPUT  SHAPE :
                          --! 入力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        O_SHAPE         : --! @brief OUTPUT SHAPE :
                          --! 出力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        ELEMENT_SIZE    : --! @brief ELEMENT SIZE :
                          --! カーネル係数バッファの容量を指定する.
                          --! * ここで指定する単位は9ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --! * 9ワードは 9 * 32 = 288 bit
                          --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                          integer := (1024/32)*256;
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 1;
        QUEUE_SIZE      : --! @brief OUTPUT PIPELINE QUEUE SIZE :
                          --! パイプラインレジスタの深さを指定する.
                          --! * QUEUE_SIZE=0 の場合は出力にキューが挿入されずダイレ
                          --!   クトに出力される.
                          integer := 0;
        ID              : --! @brief SDPRAM IDENTIFIER :
                          --! どのモジュールで使われているかを示す識別番号.
                          integer := 0 
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : --! @brief INPUT C CHANNEL SIZE :
                          in  integer range 0 to I_SHAPE.C.MAX_SIZE := I_SHAPE.C.SIZE;
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  integer range 0 to O_SHAPE.C.MAX_SIZE := O_SHAPE.C.SIZE;
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  integer range 0 to O_SHAPE.X.MAX_SIZE := O_SHAPE.X.SIZE;
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  integer range 0 to O_SHAPE.Y.MAX_SIZE := O_SHAPE.Y.SIZE;
        K3x3            : --! @brief KERNEL SIZE :
                          --! * Kernel が 3x3 の場合は'1'.
                          --! * Kernel が 1x1 の場合は'0'.
                          in  std_logic;
        REQ_WRITE       : --! @brief REQUEST BUFFER WRITE :
                          in  std_logic := '1';
        REQ_READ        : --! @brief REQUEST BUFFER READ :
                          in  std_logic := '1';
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT K_DATA :
                          --! K_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        I_VALID         : --! @brief INPUT K_DATA VALID :
                          --! K_DATA 入力有効信号.
                          in  std_logic;
        I_READY         : --! @brief INPUT IN_DATA READY :
                          --! K_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT IMAGE STREAM DATA :
                          --! ストリームデータ出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT IMAGE STREAM DATA VALID :
                          --! 出力ストリームデータ有効信号.
                          out std_logic;
        O_READY         : --! @brief OUTPUT IMAGE STREAM DATA READY :
                          --! 出力ストリームデータレディ信号.
                          in  std_logic
    );
end QCONV_STRIP_K_DATA_BUFFER;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.COMPONENTS.SDPRAM;
use     PIPEWORK.COMPONENTS.PIPELINE_REGISTER;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.CONVOLUTION_TYPES.all;
use     PIPEWORK.CONVOLUTION_COMPONENTS.CONVOLUTION_PARAMETER_BUFFER_WRITER;
use     PIPEWORK.CONVOLUTION_COMPONENTS.CONVOLUTION_PARAMETER_BUFFER_READER;
architecture RTL of QCONV_STRIP_K_DATA_BUFFER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  WORD_BITS             :  integer := QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  BUF_KERN_SIZE         :  integer := 3*3;
    constant  BUF_BANK_SIZE         :  integer := BUF_KERN_SIZE*IN_C_UNROLL*OUT_C_UNROLL;
    -------------------------------------------------------------------------------
    -- BUF_WIDTH : メモリのビット幅を２のべき乗値で示す
    -------------------------------------------------------------------------------
    function  CALC_BUF_WIDTH    return integer is
        variable width              :  integer;
    begin
        width := 0;
        while (2**width < (WORD_BITS)) loop
            width := width + 1;
        end loop;
        return width;
    end function;
    constant  BUF_WIDTH             :  integer := CALC_BUF_WIDTH;
    -------------------------------------------------------------------------------
    -- BUF_DEPTH: メモリバンク１つあたりの深さ(ビット単位)を２のべき乗値で示す
    -------------------------------------------------------------------------------
    function  CALC_BUF_DEPTH    return integer is
        variable size               :  integer;
        variable depth              :  integer;
    begin
        size  := ELEMENT_SIZE*WORD_BITS;
        size  := (size + (BUF_BANK_SIZE - 1))/BUF_BANK_SIZE;
        depth := 0;
        while (2**depth < size) loop
            depth := depth + 1;
        end loop;
        return depth;
    end function;
    constant  BUF_DEPTH             :  integer := CALC_BUF_DEPTH;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  BUF_DATA_BITS         :  integer := 2**BUF_WIDTH;
    constant  BUF_ADDR_BITS         :  integer := BUF_DEPTH - BUF_WIDTH;
    constant  BUF_WENA_BITS         :  integer := 1;
    constant  BUF_SIZE_BITS         :  integer := BUF_ADDR_BITS + 1;
    signal    conv3x3_buf_wdata     :  std_logic_vector(BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv3x3_buf_waddr     :  std_logic_vector(BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    signal    conv3x3_buf_we        :  std_logic_vector(BUF_BANK_SIZE*BUF_WENA_BITS-1 downto 0);
    signal    conv3x3_buf_rdata     :  std_logic_vector(BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv3x3_buf_raddr     :  std_logic_vector(BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    signal    conv1x1_buf_wdata     :  std_logic_vector(BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv1x1_buf_waddr     :  std_logic_vector(BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    signal    conv1x1_buf_we        :  std_logic_vector(BUF_BANK_SIZE*BUF_WENA_BITS-1 downto 0);
    signal    conv1x1_buf_rdata     :  std_logic_vector(BUF_BANK_SIZE*BUF_DATA_BITS-1 downto 0);
    signal    conv1x1_buf_raddr     :  std_logic_vector(BUF_BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  CONV_BUF_INFO(DATA: std_logic_vector; O_SIZE,I_SIZE, BITS: integer) return std_logic_vector is
        alias     i_data            :  std_logic_vector(I_SIZE*OUT_C_UNROLL*IN_C_UNROLL*BITS-1 downto 0) is DATA;
        variable  o_data            :  std_logic_vector(O_SIZE*OUT_C_UNROLL*IN_C_UNROLL*BITS-1 downto 0);
    begin
        for o_pos in 0 to O_SIZE-1       loop
        for d_pos in 0 to OUT_C_UNROLL-1 loop
        for c_pos in 0 to IN_C_UNROLL -1 loop
            if (o_pos <= I_SIZE-1) then
                o_data(((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                       ((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         )
                :=
                i_data(((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                       ((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         );
            else
                o_data(((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                       ((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         )
                :=    (((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                       ((o_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         => '0');
            end if;
        end loop;
        end loop;
        end loop;
        return o_data;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  IO_SHAPE              :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                           ELEM_BITS => WORD_BITS,
                                           C         => I_SHAPE.C,
                                           D         => O_SHAPE.C,
                                           X         => O_SHAPE.X,
                                           Y         => O_SHAPE.Y
                                       );
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      STATE_TYPE            is (IDLE_STATE,RES_STATE,
                                        CONV3x3_WR_REQ_STATE,
                                        CONV3x3_WR_RES_STATE,
                                        CONV3x3_RD_REQ_STATE,
                                        CONV3x3_RD_RES_STATE,
                                        CONV1x1_WR_REQ_STATE,
                                        CONV1x1_WR_RES_STATE,
                                        CONV1x1_RD_REQ_STATE,
                                        CONV1x1_RD_RES_STATE);
    signal    state                 :  STATE_TYPE;
    signal    wr_rd                 :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    conv3x3_wr_req_valid  :  std_logic;
    signal    conv3x3_wr_req_ready  :  std_logic;
    signal    conv3x3_wr_res_valid  :  std_logic;
    signal    conv3x3_wr_res_ready  :  std_logic;
    signal    conv3x3_wr_busy       :  std_logic;
    signal    conv3x3_rd_req_valid  :  std_logic;
    signal    conv3x3_rd_req_ready  :  std_logic;
    signal    conv3x3_rd_res_valid  :  std_logic;
    signal    conv3x3_rd_res_ready  :  std_logic;
    signal    conv3x3_rd_busy       :  std_logic;
    signal    conv3x3_i_data        :  std_logic_vector(WORD_BITS-1 downto 0);
    signal    conv3x3_i_valid       :  std_logic;
    signal    conv3x3_i_ready       :  std_logic;
    signal    conv3x3_o_data        :  std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
    signal    conv3x3_o_valid       :  std_logic;
    signal    conv3x3_o_ready       :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    conv1x1_wr_req_valid  :  std_logic;
    signal    conv1x1_wr_req_ready  :  std_logic;
    signal    conv1x1_wr_res_valid  :  std_logic;
    signal    conv1x1_wr_res_ready  :  std_logic;
    signal    conv1x1_wr_busy       :  std_logic;
    signal    conv1x1_rd_req_valid  :  std_logic;
    signal    conv1x1_rd_req_ready  :  std_logic;
    signal    conv1x1_rd_res_valid  :  std_logic;
    signal    conv1x1_rd_res_ready  :  std_logic;
    signal    conv1x1_rd_busy       :  std_logic;
    signal    conv1x1_i_data        :  std_logic_vector(WORD_BITS-1 downto 0);
    signal    conv1x1_i_valid       :  std_logic;
    signal    conv1x1_i_ready       :  std_logic;
    signal    conv1x1_o_data        :  std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
    signal    conv1x1_o_valid       :  std_logic;
    signal    conv1x1_o_ready       :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    output_data           :  std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
    signal    output_valid          :  std_logic;
    signal    output_ready          :  std_logic;
begin
    -------------------------------------------------------------------------------
    -- メインシーケンサ
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                state <= IDLE_STATE;
                wr_rd <= '0';
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                state <= IDLE_STATE;
                wr_rd <= '0';
            else
                case state is
                    when IDLE_STATE =>
                        if    (REQ_VALID = '1' and K3x3  = '1' and REQ_WRITE = '1') then
                            state <= CONV3x3_WR_REQ_STATE;
                            wr_rd <= REQ_READ;
                        elsif (REQ_VALID = '1' and K3x3  = '1' and REQ_READ  = '1') then
                            state <= CONV3x3_RD_REQ_STATE;
                            wr_rd <= '0';
                        elsif (REQ_VALID = '1' and K3x3 /= '1' and REQ_WRITE = '1') then
                            state <= CONV1x1_WR_REQ_STATE;
                            wr_rd <= REQ_READ;
                        elsif (REQ_VALID = '1' and K3x3 /= '1' and REQ_READ  = '1') then
                            state <= CONV1x1_RD_REQ_STATE;
                            wr_rd <= '0';
                        elsif (REQ_VALID = '1') then
                            state <= RES_STATE;
                            wr_rd <= '0';
                        else
                            state <= IDLE_STATE;
                        end if;
                    when CONV3x3_WR_REQ_STATE =>
                        if (conv3x3_wr_req_ready = '1') then
                            state <= CONV3x3_WR_RES_STATE;
                        else
                            state <= CONV3x3_WR_REQ_STATE;
                        end if;
                    when CONV3x3_WR_RES_STATE =>
                        if    (conv3x3_wr_res_valid = '1' and wr_rd = '1') then
                            state <= CONV3x3_RD_REQ_STATE;
                        elsif (conv3x3_wr_res_valid = '1' and wr_rd = '0') then
                            state <= RES_STATE;
                        else
                            state <= CONV3x3_WR_RES_STATE;
                        end if;
                    when CONV3x3_RD_REQ_STATE =>
                        if (conv3x3_rd_req_ready = '1') then
                            state <= CONV3x3_RD_RES_STATE;
                        else
                            state <= CONV3x3_RD_REQ_STATE;
                        end if;
                    when CONV3x3_RD_RES_STATE =>
                        if    (conv3x3_rd_res_valid = '1') then
                            state <= RES_STATE;
                        else
                            state <= CONV3x3_RD_RES_STATE;
                        end if;
                    when CONV1x1_WR_REQ_STATE =>
                        if (conv1x1_wr_req_ready = '1') then
                            state <= CONV1x1_WR_RES_STATE;
                        else
                            state <= CONV1x1_WR_REQ_STATE;
                        end if;
                    when CONV1x1_WR_RES_STATE =>
                        if    (conv1x1_wr_res_valid = '1' and wr_rd = '1') then
                            state <= CONV1x1_RD_REQ_STATE;
                        elsif (conv1x1_wr_res_valid = '1' and wr_rd = '0') then
                            state <= RES_STATE;
                        else
                            state <= CONV1x1_WR_RES_STATE;
                        end if;
                    when CONV1x1_RD_REQ_STATE =>
                        if (conv1x1_rd_req_ready = '1') then
                            state <= CONV1x1_RD_RES_STATE;
                        else
                            state <= CONV1x1_RD_REQ_STATE;
                        end if;
                    when CONV1x1_RD_RES_STATE =>
                        if    (conv1x1_rd_res_valid = '1') then
                            state <= RES_STATE;
                        else
                            state <= CONV1x1_RD_RES_STATE;
                        end if;
                    when RES_STATE =>
                        if (RES_READY = '1') then
                            state <= IDLE_STATE;
                        else
                            state <= RES_STATE;
                        end if;
                    when others =>
                            state <= IDLE_STATE;
                end case;
            end if;
        end if;
    end process;
    REQ_READY <= '1' when (state = IDLE_STATE) else '0';
    RES_VALID <= '1' when (state = RES_STATE ) else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    conv3x3_wr_req_valid <= '1' when (state = CONV3x3_WR_REQ_STATE) else '0';
    conv3x3_wr_res_ready <= '1' when (state = CONV3x3_WR_RES_STATE) else '0';
    conv3x3_rd_req_valid <= '1' when (state = CONV3x3_RD_REQ_STATE) else '0';
    conv3x3_rd_res_ready <= '1' when (state = CONV3x3_RD_RES_STATE) else '0';
    conv1x1_wr_req_valid <= '1' when (state = CONV1x1_WR_REQ_STATE) else '0';
    conv1x1_wr_res_ready <= '1' when (state = CONV1x1_WR_RES_STATE) else '0';
    conv1x1_rd_req_valid <= '1' when (state = CONV1x1_RD_REQ_STATE) else '0';
    conv1x1_rd_res_ready <= '1' when (state = CONV1x1_RD_RES_STATE) else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    conv3x3_i_data  <= I_DATA;
    conv1x1_i_data  <= I_DATA;
    conv3x3_i_valid <= '1' when (I_VALID = '1' and state = CONV3x3_WR_RES_STATE) else '0';
    conv1x1_i_valid <= '1' when (I_VALID = '1' and state = CONV1x1_WR_RES_STATE) else '0';
    I_READY <= '1' when (conv3x3_i_ready = '1' and state = CONV3x3_WR_RES_STATE) or
                        (conv1x1_i_ready = '1' and state = CONV1x1_WR_RES_STATE) else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    CONV3x3: block
        constant  KERNEL_SIZE   :  CONVOLUTION_KERNEL_SIZE_TYPE := CONVOLUTION_KERNEL_SIZE_3x3;
        constant  BUF_PARAM     :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD             ,
                                       C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(IN_C_UNROLL                      ),
                                       D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(OUT_C_UNROLL                     ),
                                       X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.X.LO,KERNEL_SIZE.X.HI),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.Y.LO,KERNEL_SIZE.Y.HI)
                                   );
        constant  buf_wready    :  std_logic := '1';
        signal    rd_req_addr   :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
        signal    wr_res_addr   :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
        signal    wr_res_size   :  std_logic_vector(BUF_SIZE_BITS-1 downto 0);
        signal    buf_out_data  :  std_logic_vector(BUF_PARAM.DATA.SIZE-1 downto 0);
    begin
        ---------------------------------------------------------------------------
        -- WRITER
        ---------------------------------------------------------------------------
        WR: CONVOLUTION_PARAMETER_BUFFER_WRITER          -- 
            generic map (                                -- 
                PARAM           => BUF_PARAM           , --
                SHAPE           => IO_SHAPE            , --
                BANK_SIZE       => BUF_BANK_SIZE       , --
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS         --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 制御 I/F
            -----------------------------------------------------------------------
                REQ_VALID       => conv3x3_wr_req_valid, -- In  :
                REQ_READY       => conv3x3_wr_req_ready, -- out :
                C_SIZE          => IN_C_BY_WORD        , -- In  :
                D_SIZE          => OUT_C               , -- In  :
                RES_VALID       => conv3x3_wr_res_valid, -- Out :
                RES_READY       => conv3x3_wr_res_ready, -- In  :
                RES_ADDR        => wr_res_addr         , -- Out :
                RES_SIZE        => wr_res_size         , -- Out :
                BUSY            => conv3x3_wr_busy     , -- Out :
            -----------------------------------------------------------------------
            -- 入力 I/F
            -----------------------------------------------------------------------
                I_DATA          => conv3x3_i_data      , -- In  :
                I_VALID         => conv3x3_i_valid     , -- In  :
                I_READY         => conv3x3_i_ready     , -- Out :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => conv3x3_buf_wdata   , -- Out :
                BUF_ADDR        => conv3x3_buf_waddr   , -- Out :
                BUF_WE          => conv3x3_buf_we      , -- Out :
                BUF_PUSH        => open                , -- Out :
                BUF_READY       => buf_wready            -- In  :
            );                                           --  
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    rd_req_addr <= (others => '0');
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    rd_req_addr <= (others => '0');
                elsif (conv3x3_wr_res_valid = '1' and conv3x3_wr_res_ready = '1') then
                    rd_req_addr <= wr_res_addr;
                end if;
            end if;
        end process;
        ---------------------------------------------------------------------------
        -- READER
        ---------------------------------------------------------------------------
        RD: CONVOLUTION_PARAMETER_BUFFER_READER          -- 
            generic map (                                -- 
                PARAM           => BUF_PARAM           , -- 
                SHAPE           => IO_SHAPE            , --
                BANK_SIZE       => BUF_BANK_SIZE       , -- 
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS         --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 制御 I/F
            -----------------------------------------------------------------------
                REQ_VALID       => conv3x3_rd_req_valid, -- In  :
                REQ_READY       => conv3x3_rd_req_ready, -- out :
                REQ_ADDR        => rd_req_addr         , -- In  :
                REQ_ADDR_LOAD   => wr_rd               , -- In  :
                C_SIZE          => IN_C_BY_WORD        , -- In  :
                D_SIZE          => OUT_C               , -- In  :
                X_SIZE          => OUT_W               , -- In  :
                Y_SIZE          => OUT_H               , -- In  :
                RES_VALID       => conv3x3_rd_res_valid, -- Out :
                RES_READY       => conv3x3_rd_res_ready, -- In  :
                BUSY            => conv3x3_rd_busy     , -- Out :
            -----------------------------------------------------------------------
            -- 出力側 I/F
            -----------------------------------------------------------------------
                O_DATA          => buf_out_data        , -- Out :
                O_VALID         => conv3x3_o_valid     , -- Out :
                O_READY         => conv3x3_o_ready     , -- In  :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => conv3x3_buf_rdata   , -- In  :
                BUF_ADDR        => conv3x3_buf_raddr     -- Out :
            );                                           -- 
        ---------------------------------------------------------------------------
        -- conv3x3_o_data
        ---------------------------------------------------------------------------
        conv3x3_o_data <= CONVOLUTION_PIPELINE_FROM_WEIGHT_STREAM(O_PARAM, BUF_PARAM, KERNEL_SIZE, buf_out_data);
    end block;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    CONV1x1: block
        constant  KERNEL_SIZE   :  CONVOLUTION_KERNEL_SIZE_TYPE := CONVOLUTION_KERNEL_SIZE_1x1;
        constant  KERN_SIZE     :  integer  := 8;
        constant  BUF_PARAM     :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD,
                                       C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(IN_C_UNROLL*KERN_SIZE            ),
                                       D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(OUT_C_UNROLL                     ),
                                       X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.X.LO,KERNEL_SIZE.X.HI),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(KERNEL_SIZE.Y.LO,KERNEL_SIZE.Y.HI)
                                   );
        constant  BANK_SIZE     :  integer  := KERN_SIZE*IN_C_UNROLL*OUT_C_UNROLL;
        signal    buf_wdata     :  std_logic_vector(BANK_SIZE*BUF_DATA_BITS-1 downto 0);
        signal    buf_waddr     :  std_logic_vector(BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
        signal    buf_we        :  std_logic_vector(BANK_SIZE*BUF_WENA_BITS-1 downto 0);
        signal    buf_rdata     :  std_logic_vector(BANK_SIZE*BUF_DATA_BITS-1 downto 0);
        signal    buf_raddr     :  std_logic_vector(BANK_SIZE*BUF_ADDR_BITS-1 downto 0);
        constant  buf_wready    :  std_logic := '1';
        signal    rd_req_addr   :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
        signal    wr_res_addr   :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
        signal    wr_res_size   :  std_logic_vector(BUF_SIZE_BITS  -1 downto 0);
        signal    buf_out_data  :  std_logic_vector(BUF_PARAM.DATA.SIZE-1 downto 0);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        function  TO_BUF_INFO(DATA: std_logic_vector; BITS: integer) return std_logic_vector is
            alias     i_data    :  std_logic_vector(    KERN_SIZE*OUT_C_UNROLL*IN_C_UNROLL*BITS-1 downto 0) is DATA;
            variable  o_data    :  std_logic_vector(BUF_KERN_SIZE*OUT_C_UNROLL*IN_C_UNROLL*BITS-1 downto 0);
        begin
            for k_pos in 0 to BUF_KERN_SIZE-1 loop
            for d_pos in 0 to OUT_C_UNROLL -1 loop
            for c_pos in 0 to IN_C_UNROLL  -1 loop
                if (k_pos < KERN_SIZE) then
                    o_data(((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                           ((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         )
                    :=
                    i_data(((d_pos*KERN_SIZE*IN_C_UNROLL   ) + (k_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                           ((d_pos*KERN_SIZE*IN_C_UNROLL   ) + (k_pos*IN_C_UNROLL) + c_pos    )*BITS         );
                else
                    o_data(((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                           ((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         )
                    :=    (((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                           ((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         => '0');
                end if;
            end loop;
            end loop;
            end loop;
            return o_data;
        end function;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        function  FROM_BUF_INFO(DATA: std_logic_vector; BITS: integer) return std_logic_vector is
            alias     i_data    :  std_logic_vector(BUF_KERN_SIZE*OUT_C_UNROLL*IN_C_UNROLL*BITS-1 downto 0) is DATA;
            variable  o_data    :  std_logic_vector(    KERN_SIZE*OUT_C_UNROLL*IN_C_UNROLL*BITS-1 downto 0);
        begin
            for k_pos in 0 to KERN_SIZE    -1 loop
            for d_pos in 0 to OUT_C_UNROLL -1 loop
            for c_pos in 0 to IN_C_UNROLL  -1 loop
                    o_data(((d_pos*KERN_SIZE*IN_C_UNROLL   ) + (k_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                           ((d_pos*KERN_SIZE*IN_C_UNROLL   ) + (k_pos*IN_C_UNROLL) + c_pos    )*BITS         )
                    :=
                    i_data(((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos + 1)*BITS-1 downto
                           ((k_pos*OUT_C_UNROLL*IN_C_UNROLL) + (d_pos*IN_C_UNROLL) + c_pos    )*BITS         );
            end loop;
            end loop;
            end loop;
            return o_data;
        end function;
    begin
        ---------------------------------------------------------------------------
        -- WRITER
        ---------------------------------------------------------------------------
        WR: CONVOLUTION_PARAMETER_BUFFER_WRITER          -- 
            generic map (                                -- 
                PARAM           => BUF_PARAM           , --
                SHAPE           => IO_SHAPE            , --
                BANK_SIZE       => BANK_SIZE           , --
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS         --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 制御 I/F
            -----------------------------------------------------------------------
                REQ_VALID       => conv1x1_wr_req_valid, -- In  :
                REQ_READY       => conv1x1_wr_req_ready, -- out :
                C_SIZE          => IN_C_BY_WORD        , -- In  :
                D_SIZE          => OUT_C               , -- In  :
                RES_VALID       => conv1x1_wr_res_valid, -- Out :
                RES_READY       => conv1x1_wr_res_ready, -- In  :
                RES_ADDR        => wr_res_addr         , -- Out :
                RES_SIZE        => wr_res_size         , -- Out :
                BUSY            => conv1x1_wr_busy     , -- Out :
            -----------------------------------------------------------------------
            -- 入力 I/F
            -----------------------------------------------------------------------
                I_DATA          => conv1x1_i_data      , -- In  :
                I_VALID         => conv1x1_i_valid     , -- In  :
                I_READY         => conv1x1_i_ready     , -- Out :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => buf_wdata           , -- Out :
                BUF_ADDR        => buf_waddr           , -- Out :
                BUF_WE          => buf_we              , -- Out :
                BUF_PUSH        => open                , -- Out :
                BUF_READY       => buf_wready            -- In  :
            );                                           --  
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    rd_req_addr <= (others => '0');
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    rd_req_addr <= (others => '0');
                elsif (conv1x1_wr_res_valid = '1' and conv1x1_wr_res_ready = '1') then
                    rd_req_addr <= wr_res_addr;
                end if;
            end if;
        end process;
        ---------------------------------------------------------------------------
        -- READER
        ---------------------------------------------------------------------------
        RD: CONVOLUTION_PARAMETER_BUFFER_READER          -- 
            generic map (                                -- 
                PARAM           => BUF_PARAM           , -- 
                SHAPE           => IO_SHAPE            , --
                BANK_SIZE       => BANK_SIZE           , -- 
                BUF_ADDR_BITS   => BUF_ADDR_BITS       , --
                BUF_DATA_BITS   => BUF_DATA_BITS         --
            )                                            -- 
            port map (                                   -- 
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK             => CLK                 , -- In  :
                RST             => RST                 , -- In  :
                CLR             => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 制御 I/F
            -----------------------------------------------------------------------
                REQ_VALID       => conv1x1_rd_req_valid, -- In  :
                REQ_READY       => conv1x1_rd_req_ready, -- Out :
                REQ_ADDR        => rd_req_addr         , -- In  :
                REQ_ADDR_LOAD   => wr_rd               , -- In  :
                C_SIZE          => IN_C_BY_WORD        , -- In  :
                D_SIZE          => OUT_C               , -- In  :
                X_SIZE          => OUT_W               , -- In  :
                Y_SIZE          => OUT_H               , -- In  :
                RES_VALID       => conv1x1_rd_res_valid, -- Out :
                RES_READY       => conv1x1_rd_res_ready, -- In  :
                BUSY            => conv1x1_rd_busy     , -- Out :
            -----------------------------------------------------------------------
            -- 出力側 I/F
            -----------------------------------------------------------------------
                O_DATA          => buf_out_data        , -- Out :
                O_VALID         => conv1x1_o_valid     , -- Out :
                O_READY         => conv1x1_o_ready     , -- In  :
            -----------------------------------------------------------------------
            -- バッファメモリ I/F
            -----------------------------------------------------------------------
                BUF_DATA        => buf_rdata           , -- In  :
                BUF_ADDR        => buf_raddr             -- Out :
            );                                           -- 
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv1x1_buf_wdata <= TO_BUF_INFO(buf_wdata, BUF_DATA_BITS);
        conv1x1_buf_waddr <= TO_BUF_INFO(buf_waddr, BUF_ADDR_BITS);
        conv1x1_buf_we    <= TO_BUF_INFO(buf_we   , BUF_WENA_BITS);
        conv1x1_buf_raddr <= TO_BUF_INFO(buf_raddr, BUF_ADDR_BITS);
        buf_rdata <= FROM_BUF_INFO(conv1x1_buf_rdata, BUF_DATA_BITS);
        ---------------------------------------------------------------------------
        -- conv1x1_o_data
        ---------------------------------------------------------------------------
        conv1x1_o_data <= CONVOLUTION_PIPELINE_FROM_WEIGHT_STREAM(O_PARAM, BUF_PARAM, KERNEL_SIZE, buf_out_data);
    end block;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    output_data     <= conv3x3_o_data when (state = CONV3x3_RD_RES_STATE) else conv1x1_o_data;
    output_valid    <= '1' when (conv3x3_o_valid = '1' and state = CONV3x3_RD_RES_STATE) or
                                (conv1x1_o_valid = '1' and state = CONV1x1_RD_RES_STATE) else '0';
    conv3x3_o_ready <= '1' when (output_ready    = '1' and state = CONV3x3_RD_RES_STATE) else '0';
    conv1x1_o_ready <= '1' when (output_ready    = '1' and state = CONV1x1_RD_RES_STATE) else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    BUF: for bank in 0 to BUF_BANK_SIZE-1 generate
        constant  RAM_ID :  integer := ID + bank;
        signal    wdata  :  std_logic_vector(BUF_DATA_BITS-1 downto 0);
        signal    waddr  :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
        signal    we     :  std_logic_vector(BUF_WENA_BITS-1 downto 0);
        signal    rdata  :  std_logic_vector(BUF_DATA_BITS-1 downto 0);
        signal    raddr  :  std_logic_vector(BUF_ADDR_BITS-1 downto 0);
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        wdata <= conv3x3_buf_wdata((bank+1)*BUF_DATA_BITS-1 downto (bank)*BUF_DATA_BITS) or
                 conv1x1_buf_wdata((bank+1)*BUF_DATA_BITS-1 downto (bank)*BUF_DATA_BITS);
        waddr <= conv3x3_buf_waddr((bank+1)*BUF_ADDR_BITS-1 downto (bank)*BUF_ADDR_BITS) or
                 conv1x1_buf_waddr((bank+1)*BUF_ADDR_BITS-1 downto (bank)*BUF_ADDR_BITS);
        we    <= conv3x3_buf_we   ((bank+1)*BUF_WENA_BITS-1 downto (bank)*BUF_WENA_BITS) or
                 conv1x1_buf_we   ((bank+1)*BUF_WENA_BITS-1 downto (bank)*BUF_WENA_BITS);
        raddr <= conv3x3_buf_raddr((bank+1)*BUF_ADDR_BITS-1 downto (bank)*BUF_ADDR_BITS) or
                 conv1x1_buf_raddr((bank+1)*BUF_ADDR_BITS-1 downto (bank)*BUF_ADDR_BITS);
        conv3x3_buf_rdata((bank+1)*BUF_DATA_BITS-1 downto (bank)*BUF_DATA_BITS) <= rdata;
        conv1x1_buf_rdata((bank+1)*BUF_DATA_BITS-1 downto (bank)*BUF_DATA_BITS) <= rdata;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        RAM: SDPRAM                   -- 
            generic map (             -- 
                DEPTH   => BUF_DEPTH, -- メモリの深さ(ビット単位)を2のべき乗値で指定する.
                RWIDTH  => BUF_WIDTH, -- リードデータ(RDATA)の幅(ビット数)を2のべき乗値で指定する.
                WWIDTH  => BUF_WIDTH, -- ライトデータ(WDATA)の幅(ビット数)を2のべき乗値で指定する.
                WEBIT   => 0        , -- ライトイネーブル信号(WE)の幅(ビット数)を2のべき乗値で指定する.
                ID      => RAM_ID     -- どのモジュールで使われているかを示す識別番号.
            )                         -- 
            port map (                -- 
                WCLK    => CLK      , -- In  :
                WE      => we       , -- In  : 
                WADDR   => waddr    , -- In  : 
                WDATA   => wdata    , -- In  : 
                RCLK    => CLK      , -- In  :
                RADDR   => raddr    , -- In  :
                RDATA   => rdata      -- Out :
            );                        -- 
    end generate;
    -------------------------------------------------------------------------------
    -- パイプラインレジスタ
    -------------------------------------------------------------------------------
    QUEUE: PIPELINE_REGISTER                   -- 
        generic map (                          -- 
            QUEUE_SIZE  => QUEUE_SIZE        , --
            WORD_BITS   => O_PARAM.DATA.SIZE   -- 
        )                                      -- 
        port map (                             -- 
            CLK         => CLK               , -- In  :
            RST         => RST               , -- In  :
            CLR         => CLR               , -- In  :
            I_WORD      => output_data       , -- In  :
            I_VAL       => output_valid      , -- In  :
            I_RDY       => output_ready      , -- Out :
            Q_WORD      => O_DATA            , -- Out :
            Q_VAL       => O_VALID           , -- Out :
            Q_RDY       => O_READY           , -- In  :
            BUSY        => open                -- Out :
        );                                     -- 
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_th_data_buffer.vhd
--!     @brief   Quantized Convolution (strip) Thresholds Data Buffer Module
--!     @version 0.1.0
--!     @date    2019/4/25
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_TH_DATA_BUFFER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        O_PARAM         : --! @brief OUTPUT STREAM PARAMETER :
                          --! 出力側の IMAGE STREAM のパラメータを指定する.
                          IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                              ELEM_BITS => 64,
                              C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(3*3),
                              D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                              Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                          );
        O_SHAPE         : --! @brief OUTPUT SHAPE :
                          --! 出力側のイメージの形(SHAPE)を指定する.
                          IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_EXTERNAL(64,1024,1024,1024);
        ELEMENT_SIZE    : --! @brief ELEMENT SIZE :
                          --! THRESHOLDS バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS
                          --! * = 64bit
                          integer := 256;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 1;
        ID              : --! @brief SDPRAM IDENTIFIER :
                          --! どのモジュールで使われているかを示す識別番号.
                          integer := 0 
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  integer range 0 to O_SHAPE.C.MAX_SIZE := O_SHAPE.C.SIZE;
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  integer range 0 to O_SHAPE.X.MAX_SIZE := O_SHAPE.X.SIZE;
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  integer range 0 to O_SHAPE.Y.MAX_SIZE := O_SHAPE.Y.SIZE;
        REQ_WRITE       : --! @brief REQUEST BUFFER WRITE :
                          in  std_logic := '1';
        REQ_READ        : --! @brief REQUEST BUFFER READ :
                          in  std_logic := '1';
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 入力側 I/F
    -------------------------------------------------------------------------------
        I_DATA          : --! @brief INPUT THRESHOLDS DATA :
                          --! THRESHOLDS DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS-1 downto 0);
        I_VALID         : --! @brief INPUT THRESHOLDS DATA VALID :
                          --! THRESHOLDS DATA 入力有効信号.
                          in  std_logic;
        I_READY         : --! @brief INPUT THRESHOLDS READY :
                          --! THRESHOLDS DATA 入力レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- 出力側 I/F
    -------------------------------------------------------------------------------
        O_DATA          : --! @brief OUTPUT THRESHOLDS DATA :
                          --! THRESHOLDS DATA 出力.
                          out std_logic_vector(O_PARAM.DATA.SIZE-1 downto 0);
        O_VALID         : --! @brief OUTPUT THRESHOLDS DATA VALID :
                          --! THRESHOLDS DATA 出力有効信号.
                          out std_logic;
        O_READY         : --! @brief OUTPUT THRESHOLDS DATA READY :
                          --! THRESHOLDS DATA 出力レディ信号.
                          in  std_logic
    );
end QCONV_STRIP_TH_DATA_BUFFER;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.COMPONENTS.SDPRAM;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.CONVOLUTION_TYPES.all;
use     PIPEWORK.CONVOLUTION_COMPONENTS.CONVOLUTION_PARAMETER_BUFFER;
architecture RTL of QCONV_STRIP_TH_DATA_BUFFER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  BUF_PARAM         :  IMAGE_STREAM_PARAM_TYPE := NEW_IMAGE_STREAM_PARAM(
                                       ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS,
                                       C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(OUT_C_UNROLL, TRUE , TRUE),
                                       D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1           , FALSE, TRUE),
                                       X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1           , TRUE , TRUE),
                                       Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1           , TRUE , TRUE)
                                   );
    constant  BUF_SHAPE         :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                       ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS,
                                       C         => O_SHAPE.C,
                                       D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                                       X         => O_SHAPE.X,
                                       Y         => O_SHAPE.Y
                                   );
    signal    buf_out_data      :  std_logic_vector(BUF_PARAM.DATA.SIZE-1 downto 0);
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    BUF: CONVOLUTION_PARAMETER_BUFFER            -- 
        generic map (                            -- 
            PARAM           => BUF_PARAM       , -- 
            SHAPE           => BUF_SHAPE       , --   
            ELEMENT_SIZE    => ELEMENT_SIZE    , --   
            ID              => ID                --   
        )                                        -- 
        port map (                               -- 
        ---------------------------------------------------------------------------
        -- クロック&リセット信号
        ---------------------------------------------------------------------------
            CLK             => CLK             , --   
            RST             => RST             , --   
            CLR             => CLR             , --   
        ---------------------------------------------------------------------------
        -- 制御 I/F
        ---------------------------------------------------------------------------
            REQ_VALID       => REQ_VALID       , --   
            REQ_READY       => REQ_READY       , --   
            REQ_WRITE       => REQ_WRITE       , --   
            REQ_READ        => REQ_READ        , --   
            C_SIZE          => OUT_C           , --   
            X_SIZE          => OUT_W           , --   
            Y_SIZE          => OUT_H           , --   
            RES_VALID       => RES_VALID       , --   
            RES_READY       => RES_READY       , --   
        ---------------------------------------------------------------------------
        -- 入力 I/F
        ---------------------------------------------------------------------------
            I_DATA          => I_DATA          , --   
            I_VALID         => I_VALID         , --   
            I_READY         => I_READY         , --   
        ---------------------------------------------------------------------------
        -- 出力側 I/F
        ---------------------------------------------------------------------------
            O_DATA          => buf_out_data    , --   
            O_VALID         => O_VALID         , --   
            O_READY         => O_READY           --   
        );                                       -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    O_DATA <= CONVOLUTION_PIPELINE_FROM_BIAS_STREAM(O_PARAM, BUF_PARAM, buf_out_data);
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_controller.vhd
--!     @brief   Quantized Convolution (strip) Controller Module
--!     @version 0.1.0
--!     @date    2019/4/18
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_CONTROLLER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        IN_BUF_SIZE     : --! @brief IN DATA BUFFER SIZE :
                          --! 入力バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --!   = 64 bit.
                          --! * 入力バッファの容量は 入力チャネル × イメージの幅.
                          integer := 512*4*1;  -- 512word × BANK_SIZE × IN_C_UNROLL 
        K_BUF_SIZE      : --! @brief K DATA BUFFER SIZE :
                          --! カーネル係数バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは 3 * 3 * QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                          integer := 512*3*3*16*1;  -- 512word × 3 × 3 × OUT_C_UNROLL × IN_C_UNROLL
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1
    );
    port(
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Register Interface
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        IN_W            : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        IN_H            : in  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        OUT_C           : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        OUT_W           : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        OUT_H           : in  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        K_W             : in  std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        K_H             : in  std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        PAD_SIZE        : in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        USE_TH          : in  std_logic;
        REQ_VALID       : in  std_logic;
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_READY       : in  std_logic;
        RES_STATUS      : out std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Core Module Interface
    -------------------------------------------------------------------------------
        CORE_IN_C       : out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        CORE_IN_W       : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        CORE_IN_H       : out std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        CORE_OUT_C      : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        CORE_OUT_W      : out std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        CORE_OUT_H      : out std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        CORE_K_W        : out std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        CORE_K_H        : out std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        CORE_L_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_R_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_T_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_B_PAD_SIZE : out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        CORE_USE_TH     : out std_logic;
        CORE_PARAM_IN   : out std_logic;
        CORE_REQ_VALID  : out std_logic;
        CORE_REQ_READY  : in  std_logic;
        CORE_RES_VALID  : in  std_logic;
        CORE_RES_READY  : out std_logic;
        CORE_RES_STATUS : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In Data AXI Reader Module Interface
    -------------------------------------------------------------------------------
        I_IN_C          : out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        I_IN_W          : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_IN_H          : out std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        I_X_POS         : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_X_SIZE        : out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_REQ_VALID     : out std_logic;
        I_REQ_READY     : in  std_logic;
        I_RES_VALID     : in  std_logic;
        I_RES_READY     : out std_logic;
        I_RES_NONE      : in  std_logic;
        I_RES_ERROR     : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Kernel Weight Data AXI Reader Module Interface
    -------------------------------------------------------------------------------        
        K_IN_C          : out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        K_OUT_C         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_OUT_C_POS     : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_OUT_C_SIZE    : out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_REQ_K3x3      : out std_logic;
        K_REQ_VALID     : out std_logic;
        K_REQ_READY     : in  std_logic;
        K_RES_VALID     : in  std_logic;
        K_RES_READY     : out std_logic;
        K_RES_NONE      : in  std_logic;
        K_RES_ERROR     : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Thresholds Data AXI Reader Module Interface
    -------------------------------------------------------------------------------        
        T_OUT_C         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        T_OUT_C_POS     : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        T_OUT_C_SIZE    : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        T_REQ_VALID     : out std_logic;
        T_REQ_READY     : in  std_logic;
        T_RES_VALID     : in  std_logic;
        T_RES_READY     : out std_logic;
        T_RES_NONE      : in  std_logic;
        T_RES_ERROR     : in  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Data AXI Writer Module Interface
    -------------------------------------------------------------------------------
        O_OUT_C         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        O_OUT_W         : out std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        O_OUT_H         : out std_logic_vector(QCONV_PARAM.OUT_H_BITS-1 downto 0);
        O_C_POS         : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        O_C_SIZE        : out std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        O_X_POS         : out std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        O_X_SIZE        : out std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        O_USE_TH        : out std_logic;
        O_REQ_VALID     : out std_logic;
        O_REQ_READY     : in  std_logic;
        O_RES_VALID     : in  std_logic;
        O_RES_READY     : out std_logic;
        O_RES_NONE      : in  std_logic;
        O_RES_ERROR     : in  std_logic
    );
end QCONV_STRIP_CONTROLLER;
-----------------------------------------------------------------------------------
-- アーキテクチャ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_SLICE_RANGE_GENERATOR;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
architecture RTL of QCONV_STRIP_CONTROLLER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function CALC_BITS(SIZE:integer) return integer is
        variable bits : integer;
    begin
        bits := 0;
        while (2**bits < SIZE) loop
            bits := bits + 1;
        end loop;
        return bits;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function MIN(A,B:integer) return integer is
    begin
        if (A < B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    conv_3x3              :  boolean;
    signal    kernel_half_size      :  integer range 0 to 1;
    signal    in_data_c_by_word     :  integer range 0 to QCONV_PARAM.MAX_IN_C_BY_WORD;
    signal    in_data_x_size        :  integer range 0 to QCONV_PARAM.MAX_IN_W;
    signal    in_data_y_size        :  integer range 0 to QCONV_PARAM.MAX_IN_H;
    signal    in_data_pad_size      :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
    signal    out_data_c_size       :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
    signal    out_data_x_size       :  integer range 0 to QCONV_PARAM.MAX_OUT_W;
    signal    out_data_y_size       :  integer range 0 to QCONV_PARAM.MAX_OUT_H;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    in_data_slice_x_pos   :  integer range 0 to QCONV_PARAM.MAX_IN_W;
    signal    in_data_slice_x_size  :  integer range 0 to QCONV_PARAM.MAX_IN_W;
    signal    in_data_slice_y_pos   :  integer range 0 to QCONV_PARAM.MAX_IN_H;
    signal    in_data_slice_y_size  :  integer range 0 to QCONV_PARAM.MAX_IN_H;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    remain_out_c_size     :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
    signal    out_data_slice_c_max  :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
    signal    out_data_slice_c_pos  :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
    signal    out_data_slice_c_size :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
    signal    out_data_slice_c_last :  boolean;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    remain_out_x_size     :  integer range 0 to QCONV_PARAM.MAX_OUT_W;
    signal    out_data_slice_x_max  :  integer range 0 to QCONV_PARAM.MAX_OUT_W;
    signal    out_data_slice_x_pos  :  integer range 0 to QCONV_PARAM.MAX_OUT_W;
    signal    out_data_slice_x_size :  integer range 0 to QCONV_PARAM.MAX_OUT_W;
    signal    out_data_slice_x_first:  boolean;
    signal    out_data_slice_x_last :  boolean;
    signal    left_pad_size         :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
    signal    right_pad_size        :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
    signal    top_pad_size          :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
    signal    bottom_pad_size       :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      EXT_STATE_TYPE        is (EXT_IDLE_STATE, EXT_REQ_STATE, EXT_RES_STATE);
    signal    conv_start            :  std_logic;
    signal    conv_busy             :  std_logic;
    signal    conv_core_start       :  std_logic;
    signal    conv_core_busy        :  std_logic;
    signal    in_data_read_start    :  std_logic;
    signal    in_data_read_busy     :  std_logic;
    signal    k_data_read_start     :  std_logic;
    signal    k_data_read_busy      :  std_logic;
    signal    th_data_read_start    :  std_logic;
    signal    th_data_read_busy     :  std_logic;
    signal    out_data_write_start  :  std_logic;
    signal    out_data_write_busy   :  std_logic;
begin
    -------------------------------------------------------------------------------
    -- Top Level State Machine
    -------------------------------------------------------------------------------
    TOP: block
        ---------------------------------------------------------------------------
        -- K3x3 の場合、バッファの深さは   IN_C_UNROLL ワード単位でしか格納できない.
        -- K1x1 の場合、バッファの深さは 8*IN_C_UNROLL ワード単位でしか格納できない.
        -- ここでは入力された IN_C_BY_WORD を K3x3 の場合は IN_C_UNROLL 単位で、
        -- K1x1 の場合は 8*IN_C_UNROLL 切り上げる.
        -- 例えば K3x3 で IN_C_UNROLL が4 だった場合、
        --     IN_C_BY_WORD=1〜 4 -> ROUND_UP_IN_C_BY_WORD= 4
        --     IN_C_BY_WORD=5〜 8 -> ROUND_UP_IN_C_BY_WORD= 8
        --     IN_C_BY_WORD=9〜12 -> ROUND_UP_IN_C_BY_WORD=12
        -- rndup_in_c_by_word <= (IN_C_BY_WORD + ROUND_UP_MASK) and (not ROUND_UP_MASK);
        ---------------------------------------------------------------------------
        function  ROUND_UP_IN_C_BY_WORD(IN_C_BY_WORD: integer; C_UNROLL: integer) return integer is
            constant  ROUND_UP_MASK     :  unsigned(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0)
                                        := to_unsigned((2**CALC_BITS(C_UNROLL)-1), QCONV_PARAM.IN_C_BY_WORD_BITS);
            variable  u_in_c_by_word    :  unsigned(        QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
            variable  s_in_c_by_word    :  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
            variable  i_in_c_by_word    :  integer range 0 to QCONV_PARAM.MAX_IN_C_BY_WORD;
        begin
            u_in_c_by_word := to_unsigned(IN_C_BY_WORD, QCONV_PARAM.IN_C_BY_WORD_BITS) + ROUND_UP_MASK;
            s_in_c_by_word := std_logic_vector(u_in_c_by_word) and (not std_logic_vector(ROUND_UP_MASK));
            i_in_c_by_word := to_integer(unsigned(s_in_c_by_word));
            return i_in_c_by_word;
        end function;
        ---------------------------------------------------------------------------
        -- 各バッファの深さ
        ---------------------------------------------------------------------------
        constant  IN_BUF_DEPTH          :  integer := IN_BUF_SIZE/(    IN_C_UNROLL);
        constant  K_BUF_DEPTH           :  integer := K_BUF_SIZE /(3*3*IN_C_UNROLL);
        ---------------------------------------------------------------------------
        -- バッファの深さを IN_C_BY_WORD で割る.
        -- ただし IN_CY_B_WORD は２のべき乗値に切り上げてから BUF_DEPTH を割る.
        --     IN_C_BY_WORD= 1     -> BUF_DEPTH/1
        --     IN_C_BY_WORD= 2     -> BUF_DEPTH/2
        --     IN_C_BY_WORD= 3〜 4 -> BUF_DEPTH/4
        --     IN_C_BY_WORD= 5〜 8 -> BUF_DEPTH/8
        --     IN_C_BY_WORD= 9〜16 -> BUF_DEPTH/16
        --     IN_C_BY_WORD=17〜32 -> BUF_DEPTH/32
        ---------------------------------------------------------------------------
        function  DEVIDE_BY_LOG2(BUF_DEPTH, IN_C_BY_WORD, MAX: integer) return integer is
            variable  u_in_c_by_word    :  unsigned(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        begin
            u_in_c_by_word := to_unsigned(IN_C_BY_WORD, QCONV_PARAM.IN_C_BY_WORD_BITS) - 1;
            for i in u_in_c_by_word'high downto u_in_c_by_word'low loop
                if (u_in_c_by_word(i) = '1') then
                    return MIN(MAX, BUF_DEPTH/(2**(i+1)));
                end if;
            end loop;
            return MIN(MAX, BUF_DEPTH);
        end function;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        type      STATE_TYPE            is (IDLE_STATE, PREP_STATE, START_STATE, WAIT_STATE, NEXT_STATE, RES_STATE);
        signal    state                 :  STATE_TYPE;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST)
            variable  rndup_in_c_by_word   :  integer range 0 to QCONV_PARAM.MAX_IN_C_BY_WORD;
            variable  slice_c_max          :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
            variable  slice_x_max          :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
        begin
            if (RST = '1') then
                    state                  <= IDLE_STATE;
                    in_data_c_by_word      <= 0;
                    in_data_x_size         <= 0;
                    in_data_y_size         <= 0;
                    in_data_pad_size       <= 0;
                    remain_out_c_size      <= 0;
                    out_data_c_size        <= 0;
                    out_data_x_size        <= 0;
                    out_data_y_size        <= 0;
                    out_data_slice_c_max   <= 0;
                    out_data_slice_c_pos   <= 0;
                    out_data_slice_c_size  <= 0;
                    out_data_slice_c_last  <= FALSE;
                    out_data_slice_x_max   <= 0;
                    conv_3x3               <= FALSE;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state                  <= IDLE_STATE;
                    in_data_c_by_word      <= 0;
                    in_data_x_size         <= 0;
                    in_data_y_size         <= 0;
                    in_data_pad_size       <= 0;
                    remain_out_c_size      <= 0;
                    out_data_c_size        <= 0;
                    out_data_x_size        <= 0;
                    out_data_y_size        <= 0;
                    out_data_slice_c_max   <= 0;
                    out_data_slice_c_pos   <= 0;
                    out_data_slice_c_size  <= 0;
                    out_data_slice_c_last  <= FALSE;
                    out_data_slice_x_max   <= 0;
                    conv_3x3               <= FALSE;
                else
                    case state is
                        when IDLE_STATE =>
                            if (REQ_VALID = '1') then
                                state <= PREP_STATE;
                            else
                                state <= IDLE_STATE;
                            end if;
                            in_data_c_by_word <= to_integer(unsigned(IN_C_BY_WORD));
                            in_data_x_size    <= to_integer(unsigned(IN_W));
                            in_data_y_size    <= to_integer(unsigned(IN_H));
                            in_data_pad_size  <= to_integer(unsigned(PAD_SIZE));
                            out_data_c_size   <= to_integer(unsigned(OUT_C));
                            out_data_x_size   <= to_integer(unsigned(OUT_W));
                            out_data_y_size   <= to_integer(unsigned(OUT_H));
                            remain_out_c_size <= to_integer(unsigned(OUT_C));
                            conv_3x3 <= ((to_integer(to_01(unsigned(K_W))) = 3) and
                                         (to_integer(to_01(unsigned(K_H))) = 3));
                        when PREP_STATE =>
                            if (conv_3x3) then
                                rndup_in_c_by_word   := ROUND_UP_IN_C_BY_WORD(in_data_c_by_word, IN_C_UNROLL    );
                                out_data_slice_c_max <= DEVIDE_BY_LOG2(     K_BUF_DEPTH, rndup_in_c_by_word, QCONV_PARAM.MAX_OUT_C); 
                                out_data_slice_x_max <= DEVIDE_BY_LOG2(    IN_BUF_DEPTH, rndup_in_c_by_word, QCONV_PARAM.MAX_OUT_W) - 2;
                                kernel_half_size     <= 1;
                            else
                                rndup_in_c_by_word   := ROUND_UP_IN_C_BY_WORD(in_data_c_by_word, IN_C_UNROLL * 8);
                                out_data_slice_c_max <= DEVIDE_BY_LOG2(8 *  K_BUF_DEPTH, rndup_in_c_by_word, QCONV_PARAM.MAX_OUT_C);
                                out_data_slice_x_max <= DEVIDE_BY_LOG2(2 * IN_BUF_DEPTH, rndup_in_c_by_word, QCONV_PARAM.MAX_OUT_W);
                                kernel_half_size     <= 0;
                            end if;
                            out_data_slice_c_pos   <= 0;
                            out_data_slice_c_last  <= FALSE;
                            state <= START_STATE;
                        when START_STATE =>
                            if (remain_out_c_size <= out_data_slice_c_max) then
                                out_data_slice_c_size <= remain_out_c_size;
                                out_data_slice_c_last <= TRUE;
                            else
                                out_data_slice_c_size <= out_data_slice_c_max;
                                out_data_slice_c_last <= FALSE;
                            end if;
                            state <= WAIT_STATE;
                        when WAIT_STATE =>
                            if (k_data_read_busy = '0' and th_data_read_busy = '0' and conv_busy = '0') then
                                state <= NEXT_STATE;
                            else
                                state <= WAIT_STATE;
                            end if;
                        when NEXT_STATE =>
                            out_data_slice_c_pos <= out_data_slice_c_pos + out_data_slice_c_size;
                            remain_out_c_size    <= remain_out_c_size    - out_data_slice_c_size;
                            if (out_data_slice_c_last = TRUE) then
                                state <= RES_STATE;
                            else
                                state <= START_STATE;
                            end if;
                        when RES_STATE =>
                            if (RES_READY = '1') then
                                state <= IDLE_STATE;
                            else
                                state <= RES_STATE;
                            end if;
                        when others =>
                                state <= IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        REQ_READY  <= '1' when (state = IDLE_STATE) else '0';
        RES_VALID  <= '1' when (state = RES_STATE ) else '0';
        RES_STATUS <= '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv_start         <= '1' when (state = START_STATE) else '0';
        k_data_read_start  <= '1' when (state = START_STATE) else '0';
        th_data_read_start <= '1' when (state = START_STATE and USE_TH = '1') else '0';
    end block;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Main Control.
    -------------------------------------------------------------------------------
    CONV_CTRL: block
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        type      STATE_TYPE          is (IDLE_STATE,
                                          LOOP_START_STATE,
                                          GEN_REQ_STATE   ,
                                          GEN_RES_STATE   ,
                                          CORE_WAIT_STATE ,
                                          LOOP_NEXT_STATE);
        signal    state               :  STATE_TYPE;
        signal    gen_req_valid       :  std_logic;
        signal    gen_req_ready       :  std_logic;
        signal    gen_res_valid       :  std_logic;
        signal    gen_res_ready       :  std_logic;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        constant  IMAGE_SHAPE         :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE_CONSTANT(
                                             ELEM_BITS => 64,
                                             X         => QCONV_PARAM.MAX_IN_W,
                                             Y         => QCONV_PARAM.MAX_IN_H
                                         );
        constant  MIN_SLICE_X_POS     :  integer := - QCONV_PARAM.MAX_PAD_SIZE;
        constant  MAX_SLICE_X_POS     :  integer :=   QCONV_PARAM.MAX_IN_W;
        constant  MIN_SLICE_Y_POS     :  integer := - QCONV_PARAM.MAX_PAD_SIZE;
        constant  MAX_SLICE_Y_POS     :  integer :=   QCONV_PARAM.MAX_IN_H;
        signal    start_x_pos         :  integer range MIN_SLICE_X_POS to MAX_SLICE_X_POS;
        signal    next_x_pos          :  integer range MIN_SLICE_X_POS to MAX_SLICE_X_POS;
        signal    start_y_pos         :  integer range MIN_SLICE_Y_POS to MAX_SLICE_Y_POS;
        signal    next_y_pos          :  integer range MIN_SLICE_Y_POS to MAX_SLICE_Y_POS;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    state                 <= IDLE_STATE;
                    remain_out_x_size     <= 0;
                    out_data_slice_x_pos  <= 0;
                    out_data_slice_x_size <= 0;
                    out_data_slice_x_first<= TRUE;
                    out_data_slice_x_last <= FALSE;
                    start_x_pos           <= 0;
                    start_y_pos           <= 0;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state                 <= IDLE_STATE;
                    remain_out_x_size     <= 0;
                    out_data_slice_x_pos  <= 0;
                    out_data_slice_x_size <= 0;
                    out_data_slice_x_first<= TRUE;
                    out_data_slice_x_last <= FALSE;
                    start_x_pos           <= 0;
                    start_y_pos           <= 0;
                else
                    case state is
                        when IDLE_STATE =>
                            if (conv_start = '1') then
                                state <= LOOP_START_STATE;
                            else
                                state <= IDLE_STATE;
                            end if;
                            remain_out_x_size     <= out_data_x_size;
                            out_data_slice_x_pos  <= 0;
                            out_data_slice_x_first<= TRUE;
                            out_data_slice_x_last <= FALSE;
                            start_x_pos           <= -(in_data_pad_size - kernel_half_size);
                            start_y_pos           <= -(in_data_pad_size - kernel_half_size);
                        when LOOP_START_STATE =>
                            if (remain_out_x_size <= out_data_slice_x_max) then
                                out_data_slice_x_size <= remain_out_x_size;
                                out_data_slice_x_last <= TRUE;
                            else
                                out_data_slice_x_size <= out_data_slice_x_max;
                                out_data_slice_x_last <= FALSE;
                            end if;
                            state     <= GEN_REQ_STATE;
                        when GEN_REQ_STATE =>
                            if (gen_req_ready = '1') then
                                state <= GEN_RES_STATE;
                            else
                                state <= GEN_REQ_STATE;
                            end if;
                        when GEN_RES_STATE =>
                            if (gen_res_valid = '1') then
                                state <= CORE_WAIT_STATE;
                            else
                                state <= GEN_RES_STATE;
                            end if;
                        when CORE_WAIT_STATE =>
                            if (conv_core_busy = '0' and in_data_read_busy = '0' and out_data_write_busy = '0') then
                                state <= LOOP_NEXT_STATE;
                            else
                                state <= CORE_WAIT_STATE;
                            end if;
                        when LOOP_NEXT_STATE =>
                            start_x_pos            <= next_x_pos;
                            out_data_slice_x_first <= FALSE;
                            out_data_slice_x_pos   <= out_data_slice_x_pos + out_data_slice_x_size;
                            remain_out_x_size      <= remain_out_x_size    - out_data_slice_x_size;
                            if (out_data_slice_x_last = TRUE) then
                                state <= IDLE_STATE;
                            else
                                state <= LOOP_START_STATE;
                            end if;
                        when others =>
                                state <= IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        conv_busy            <= '1' when (state /= IDLE_STATE      ) else '0';
        conv_core_start      <= '1' when (state  = GEN_RES_STATE and gen_res_valid = '1') else '0';
        in_data_read_start   <= '1' when (state  = GEN_RES_STATE and gen_res_valid = '1') else '0';
        out_data_write_start <= '1' when (state  = GEN_RES_STATE and gen_res_valid = '1') else '0';
        gen_req_valid        <= '1' when (state  = GEN_REQ_STATE   ) else '0';
        gen_res_ready        <= '1' when (state  = LOOP_NEXT_STATE ) else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        GEN: IMAGE_SLICE_RANGE_GENERATOR
            generic map (
                SOURCE_SHAPE        => IMAGE_SHAPE         , --
                SLICE_SHAPE         => IMAGE_SHAPE         , --
                MIN_SLICE_X_POS     => MIN_SLICE_X_POS     , --
                MAX_SLICE_X_POS     => MAX_SLICE_X_POS     , --
                MIN_SLICE_Y_POS     => MIN_SLICE_Y_POS     , --
                MAX_SLICE_Y_POS     => MAX_SLICE_Y_POS     , --
                MAX_PAD_L_SIZE      => QCONV_PARAM.MAX_PAD_SIZE, --
                MAX_PAD_R_SIZE      => QCONV_PARAM.MAX_PAD_SIZE, --
                MAX_PAD_T_SIZE      => QCONV_PARAM.MAX_PAD_SIZE, --
                MAX_PAD_B_SIZE      => QCONV_PARAM.MAX_PAD_SIZE, --
                MAX_KERNEL_L_SIZE   => 1                   , --
                MAX_KERNEL_R_SIZE   => 1                   , --
                MAX_KERNEL_T_SIZE   => 1                   , --
                MAX_KERNEL_B_SIZE   => 1                     --
            )
            port map (
            -----------------------------------------------------------------------
            -- クロック&リセット信号
            -----------------------------------------------------------------------
                CLK                 => CLK                 , -- In  :
                RST                 => RST                 , -- In  :
                CLR                 => CLR                 , -- In  :
            -----------------------------------------------------------------------
            -- 計算に必要な情報
            -- これらの信号の値は計算中は変更してはならない.
            -----------------------------------------------------------------------
                SOURCE_X_SIZE       => in_data_x_size      , -- In  :
                SOURCE_Y_SIZE       => in_data_y_size      , -- In  :
                KERNEL_L_SIZE       => kernel_half_size    , -- In  :
                KERNEL_R_SIZE       => kernel_half_size    , -- In  :
                KERNEL_T_SIZE       => kernel_half_size    , -- In  :
                KERNEL_B_SIZE       => kernel_half_size    , -- In  :
            -----------------------------------------------------------------------
            -- 計算開始信号
            -----------------------------------------------------------------------
                REQ_START_X_POS     => start_x_pos         , -- In  :
                REQ_START_Y_POS     => start_y_pos         , -- In  :
                REQ_SLICE_X_SIZE    => out_data_slice_x_size,-- In  :
                REQ_SLICE_Y_SIZE    => out_data_y_size     , -- In  :
                REQ_VALID           => gen_req_valid       , -- In  :
                REQ_READY           => gen_req_ready       , -- Out :
            -----------------------------------------------------------------------
            -- 計算結果
            -----------------------------------------------------------------------
                RES_START_X_POS     => in_data_slice_x_pos , -- Out :
                RES_START_Y_POS     => in_data_slice_y_pos , -- Out :
                RES_SLICE_X_SIZE    => in_data_slice_x_size, -- Out :
                RES_SLICE_Y_SIZE    => in_data_slice_y_size, -- Out :
                RES_PAD_L_SIZE      => left_pad_size       , -- Out :
                RES_PAD_R_SIZE      => right_pad_size      , -- Out :
                RES_PAD_T_SIZE      => top_pad_size        , -- Out :
                RES_PAD_B_SIZE      => bottom_pad_size     , -- Out :
                RES_NEXT_X_POS      => next_x_pos          , -- Out :
                RES_NEXT_Y_POS      => open                , -- Out :
                RES_VALID           => gen_res_valid       , -- Out :
                RES_READY           => gen_res_ready         -- In  :
            );                                               -- 
    end block;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Core Module Control.
    -------------------------------------------------------------------------------
    CONV_CORE: block
        signal    state     :  EXT_STATE_TYPE;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    state <= EXT_IDLE_STATE;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state <= EXT_IDLE_STATE;
                else
                    case state is
                        when EXT_IDLE_STATE =>
                            if (conv_core_start = '1') then
                                state <= EXT_REQ_STATE;
                            else
                                state <= EXT_IDLE_STATE;
                            end if;
                        when EXT_REQ_STATE  =>
                            if (CORE_REQ_READY = '1') then
                                state <= EXT_RES_STATE;
                            else
                                state <= EXT_REQ_STATE;
                            end if;
                        when EXT_RES_STATE =>
                            if (CORE_RES_VALID = '1') then
                                state <= EXT_IDLE_STATE;
                            else
                                state <= EXT_RES_STATE;
                            end if;
                        when others =>
                                state <= EXT_IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        conv_core_busy <= '1' when (state /= EXT_IDLE_STATE) else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        CORE_REQ_VALID  <= '1' when (state = EXT_REQ_STATE) else '0';
        CORE_RES_READY  <= '1' when (state = EXT_RES_STATE) else '0';
        CORE_IN_C       <= std_logic_vector(to_unsigned(in_data_c_by_word    , QCONV_PARAM.IN_C_BY_WORD_BITS));
        CORE_IN_W       <= std_logic_vector(to_unsigned(in_data_slice_x_size , QCONV_PARAM.IN_W_BITS ));
        CORE_IN_H       <= std_logic_vector(to_unsigned(in_data_slice_y_size , QCONV_PARAM.IN_H_BITS ));
        CORE_OUT_C      <= std_logic_vector(to_unsigned(out_data_slice_c_size, QCONV_PARAM.OUT_C_BITS));
        CORE_OUT_W      <= std_logic_vector(to_unsigned(out_data_slice_x_size, QCONV_PARAM.OUT_W_BITS));
        CORE_OUT_H      <= std_logic_vector(to_unsigned(out_data_y_size      , QCONV_PARAM.OUT_H_BITS));
        CORE_K_W        <= K_W;
        CORE_K_H        <= K_H;
        CORE_USE_TH     <= USE_TH;
        CORE_PARAM_IN   <= '1' when (out_data_slice_x_first = TRUE) else '0';
        CORE_L_PAD_SIZE <= std_logic_vector(to_unsigned(left_pad_size        , QCONV_PARAM.PAD_SIZE_BITS));
        CORE_R_PAD_SIZE <= std_logic_vector(to_unsigned(right_pad_size       , QCONV_PARAM.PAD_SIZE_BITS));
        CORE_T_PAD_SIZE <= std_logic_vector(to_unsigned(top_pad_size         , QCONV_PARAM.PAD_SIZE_BITS));
        CORE_B_PAD_SIZE <= std_logic_vector(to_unsigned(bottom_pad_size      , QCONV_PARAM.PAD_SIZE_BITS));
    end block;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In Data AXI Reader Module Control
    -------------------------------------------------------------------------------
    IN_DATA_READ: block
        signal    state     :  EXT_STATE_TYPE;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    state <= EXT_IDLE_STATE;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state <= EXT_IDLE_STATE;
                else
                    case state is
                        when EXT_IDLE_STATE =>
                            if (in_data_read_start = '1') then
                                state <= EXT_REQ_STATE;
                            else
                                state <= EXT_IDLE_STATE;
                            end if;
                        when EXT_REQ_STATE  =>
                            if (I_REQ_READY = '1') then
                                state <= EXT_RES_STATE;
                            else
                                state <= EXT_REQ_STATE;
                            end if;
                        when EXT_RES_STATE =>
                            if (I_RES_VALID = '1') then
                                state <= EXT_IDLE_STATE;
                            else
                                state <= EXT_RES_STATE;
                            end if;
                        when others =>
                                state <= EXT_IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        in_data_read_busy <= '1' when (state /= EXT_IDLE_STATE) else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        I_REQ_VALID <= '1' when (state = EXT_REQ_STATE) else '0';
        I_RES_READY <= '1' when (state = EXT_RES_STATE) else '0';
        I_IN_C      <= std_logic_vector(to_unsigned(in_data_c_by_word   , QCONV_PARAM.IN_C_BY_WORD_BITS));
        I_IN_W      <= std_logic_vector(to_unsigned(in_data_x_size      , QCONV_PARAM.IN_W_BITS));
        I_IN_H      <= std_logic_vector(to_unsigned(in_data_slice_y_size, QCONV_PARAM.IN_H_BITS));
        I_X_POS     <= std_logic_vector(to_unsigned(in_data_slice_x_pos , QCONV_PARAM.IN_W_BITS));
        I_X_SIZE    <= std_logic_vector(to_unsigned(in_data_slice_x_size, QCONV_PARAM.IN_W_BITS));
    end block;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Kernel Weight Data AXI Reader Module Control
    -------------------------------------------------------------------------------
    K_DATA_READ: block
        signal    state     :  EXT_STATE_TYPE;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    state <= EXT_IDLE_STATE;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state <= EXT_IDLE_STATE;
                else
                    case state is
                        when EXT_IDLE_STATE =>
                            if (k_data_read_start = '1') then
                                state <= EXT_REQ_STATE;
                            else
                                state <= EXT_IDLE_STATE;
                            end if;
                        when EXT_REQ_STATE  =>
                            if (K_REQ_READY = '1') then
                                state <= EXT_RES_STATE;
                            else
                                state <= EXT_REQ_STATE;
                            end if;
                        when EXT_RES_STATE =>
                            if (K_RES_VALID = '1') then
                                state <= EXT_IDLE_STATE;
                            else
                                state <= EXT_RES_STATE;
                            end if;
                        when others =>
                                state <= EXT_IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        k_data_read_busy <= '1' when (state /= EXT_IDLE_STATE) else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        K_REQ_VALID  <= '1' when (state = EXT_REQ_STATE) else '0';
        K_RES_READY  <= '1' when (state = EXT_RES_STATE) else '0';
        K_IN_C       <= std_logic_vector(to_unsigned(in_data_c_by_word    , QCONV_PARAM.IN_C_BY_WORD_BITS));
        K_OUT_C      <= std_logic_vector(to_unsigned(out_data_c_size      , QCONV_PARAM.OUT_C_BITS));
        K_OUT_C_POS  <= std_logic_vector(to_unsigned(out_data_slice_c_pos , QCONV_PARAM.OUT_C_BITS));
        K_OUT_C_SIZE <= std_logic_vector(to_unsigned(out_data_slice_c_size, QCONV_PARAM.OUT_C_BITS));
        K_REQ_K3x3   <= '1' when (conv_3x3 = TRUE) else '0';
    end block;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Thresholds Data AXI Reader Module Control
    -------------------------------------------------------------------------------
    TH_DATA_READ: block
        signal    state     :  EXT_STATE_TYPE;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    state <= EXT_IDLE_STATE;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state <= EXT_IDLE_STATE;
                else
                    case state is
                        when EXT_IDLE_STATE =>
                            if (th_data_read_start = '1') then
                                state <= EXT_REQ_STATE;
                            else
                                state <= EXT_IDLE_STATE;
                            end if;
                        when EXT_REQ_STATE  =>
                            if (T_REQ_READY = '1') then
                                state <= EXT_RES_STATE;
                            else
                                state <= EXT_REQ_STATE;
                            end if;
                        when EXT_RES_STATE =>
                            if (T_RES_VALID = '1') then
                                state <= EXT_IDLE_STATE;
                            else
                                state <= EXT_RES_STATE;
                            end if;
                        when others =>
                                state <= EXT_IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        th_data_read_busy <= '1' when (state /= EXT_IDLE_STATE) else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        T_REQ_VALID  <= '1' when (state = EXT_REQ_STATE) else '0';
        T_RES_READY  <= '1' when (state = EXT_RES_STATE) else '0';
        T_OUT_C      <= std_logic_vector(to_unsigned(out_data_c_size      , QCONV_PARAM.OUT_C_BITS));
        T_OUT_C_POS  <= std_logic_vector(to_unsigned(out_data_slice_c_pos , QCONV_PARAM.OUT_C_BITS));
        T_OUT_C_SIZE <= std_logic_vector(to_unsigned(out_data_slice_c_size, QCONV_PARAM.OUT_C_BITS));
    end block;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Data AXI Writer Module Control
    -------------------------------------------------------------------------------
    OUT_DATA_WRITE: block
        signal    state     :  EXT_STATE_TYPE;
    begin
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        process (CLK, RST) begin
            if (RST = '1') then
                    state <= EXT_IDLE_STATE;
            elsif (CLK'event and CLK = '1') then
                if (CLR = '1') then
                    state <= EXT_IDLE_STATE;
                else
                    case state is
                        when EXT_IDLE_STATE =>
                            if (out_data_write_start = '1') then
                                state <= EXT_REQ_STATE;
                            else
                                state <= EXT_IDLE_STATE;
                            end if;
                        when EXT_REQ_STATE  =>
                            if (O_REQ_READY = '1') then
                                state <= EXT_RES_STATE;
                            else
                                state <= EXT_REQ_STATE;
                            end if;
                        when EXT_RES_STATE =>
                            if (O_RES_VALID = '1') then
                                state <= EXT_IDLE_STATE;
                            else
                                state <= EXT_RES_STATE;
                            end if;
                        when others =>
                                state <= EXT_IDLE_STATE;
                    end case;
                end if;
            end if;
        end process;
        out_data_write_busy <= '1' when (state /= EXT_IDLE_STATE) else '0';
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        O_REQ_VALID  <= '1' when (state = EXT_REQ_STATE) else '0';
        O_RES_READY  <= '1' when (state = EXT_RES_STATE) else '0';
        O_OUT_C      <= std_logic_vector(to_unsigned(out_data_c_size       , QCONV_PARAM.OUT_C_BITS));
        O_OUT_W      <= std_logic_vector(to_unsigned(out_data_x_size       , QCONV_PARAM.OUT_W_BITS));
        O_OUT_H      <= std_logic_vector(to_unsigned(out_data_y_size       , QCONV_PARAM.OUT_H_BITS));
        O_C_POS      <= std_logic_vector(to_unsigned(out_data_slice_c_pos  , QCONV_PARAM.OUT_C_BITS));
        O_C_SIZE     <= std_logic_vector(to_unsigned(out_data_slice_c_size , QCONV_PARAM.OUT_C_BITS));
        O_X_POS      <= std_logic_vector(to_unsigned(out_data_slice_x_pos  , QCONV_PARAM.OUT_W_BITS));
        O_X_SIZE     <= std_logic_vector(to_unsigned(out_data_slice_x_size , QCONV_PARAM.OUT_W_BITS));
        O_USE_TH     <= USE_TH;
    end block;
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_core.vhd
--!     @brief   Quantized Convolution (strip) Core Module
--!     @version 0.1.0
--!     @date    2019/4/25
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_CORE is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        IN_BUF_SIZE     : --! @brief IN DATA BUFFER SIZE :
                          --! 入力バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --!   = 64 bit.
                          --! * 入力バッファの容量は 入力チャネル × イメージの幅.
                          integer := 512*4*1;  -- 512word × BANK_SIZE × IN_C_UNROLL 
        K_BUF_SIZE      : --! @brief K DATA BUFFER SIZE :
                          --! カーネル係数バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは 3 * 3 * QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                          --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                          integer := 512*3*3*16*1;  -- 512word × 3 × 3 × OUT_C_UNROLL × IN_C_UNROLL
        TH_BUF_SIZE     : --! @brief THRESHOLDS DATA BUFFER SIZE :
                          --! THRESHOLDS バッファの容量を指定する.
                          --! * ここで指定する単位は1ワード単位.
                          --! * 1ワードは QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS
                          --! * = 64bit
                          integer := 512*16;
        IN_C_UNROLL     : --! @brief INPUT  CHANNEL UNROLL SIZE :
                          integer := 1;
        OUT_C_UNROLL    : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                          integer := 16;
        OUT_DATA_BITS   : --! @brief OUTPUT DATA BIT SIZE :
                          --! OUT_DATA のビット幅を指定する.
                          --! * OUT_DATA のビット幅は、64の倍数でなければならない.
                          integer := 64
    );
    port (
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
        IN_C_BY_WORD    : --! @brief INPUT C CHANNEL SIZE :
                          in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        IN_W            : --! @brief INPUT IMAGE WIDTH :
                          in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        IN_H            : --! @brief INPUT IMAGE HEIGHT :
                          in  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        OUT_C           : --! @brief OUTPUT C CHANNEL SIZE :
                          in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        OUT_W           : --! @brief OUTPUT IMAGE WIDTH :
                          in  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        OUT_H           : --! @brief OUTPUT IMAGE HEIGHT :
                          in  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        K_W             : --! @brief KERNEL WIDTH :
                          in  std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        K_H             : --! @brief KERNEL HEIGHT :
                          in  std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        LEFT_PAD_SIZE   : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        RIGHT_PAD_SIZE  : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        TOP_PAD_SIZE    : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        BOTTOM_PAD_SIZE : --! @brief PAD SIZE REGISTER :
                          in  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        USE_TH          : --! @brief USE THRESHOLD REGISTER :
                          in  std_logic;
        PARAM_IN        : --! @brief K DATA / TH DATA INPUT FLAG :
                          in  std_logic;
        REQ_VALID       : --! @brief REQUEST VALID :
                          in  std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          out std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          out std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- データ入力 I/F
    -------------------------------------------------------------------------------
        IN_DATA         : --! @brief INPUT IN_DATA :
                          --! IN_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        IN_VALID        : --! @brief INPUT IN_DATA VALID :
                          --! IN_DATA 入力有効信号.
                          in  std_logic;
        IN_READY        : --! @brief INPUT IN_DATA READY :
                          --! IN_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- カーネル係数入力 I/F
    -------------------------------------------------------------------------------
        K_DATA          : --! @brief INPUT K_DATA :
                          --! K_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        K_VALID         : --! @brief INPUT K_DATA VALID :
                          --! K_DATA 入力有効信号.
                          in  std_logic;
        K_READY         : --! @brief INPUT K_DATA READY :
                          --! K_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- スレッシュホールド係数入力 I/F
    -------------------------------------------------------------------------------
        TH_DATA         : --! @brief INPUT TH_DATA :
                          --! TH_DATA 入力.
                          in  std_logic_vector(QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS-1 downto 0);
        TH_VALID        : --! @brief INPUT TH_DATA VALID :
                          --! TH_DATA 入力有効信号.
                          in  std_logic;
        TH_READY        : --! @brief INPUT TH_DATA READY :
                          --! TH_DATA レディ信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- データ出力 I/F
    -------------------------------------------------------------------------------
        OUT_DATA        : --! @brief OUTPUT DATA :
                          --! OUT DATA 出力.
                          out std_logic_vector(OUT_DATA_BITS-1 downto 0);
        OUT_LAST        : --! @brief OUTPUT LAST DATA :
                          --! OUT LAST 出力.
                          out std_logic;
        OUT_VALID       : --! @brief OUT_DATA VALID :
                          --! OUT_DATA 出力有効信号.
                          out std_logic;
        OUT_READY       : --! @brief OUT_DATA READY :
                          --! OUT_DATA レディ信号.
                          in  std_logic
    );
end QCONV_STRIP_CORE;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
use     QCONV.QCONV_COMPONENTS.QCONV_MULTIPLIER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_IN_DATA_BUFFER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_TH_DATA_BUFFER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_K_DATA_BUFFER;
use     QCONV.QCONV_COMPONENTS.QCONV_APPLY_THRESHOLDS;
library PIPEWORK;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_STREAM_CHANNEL_REDUCER;
use     PIPEWORK.CONVOLUTION_TYPES.all;
use     PIPEWORK.CONVOLUTION_COMPONENTS.CONVOLUTION_INT_ADDER_TREE;
use     PIPEWORK.CONVOLUTION_COMPONENTS.CONVOLUTION_INT_ACCUMULATOR;
architecture RTL of QCONV_STRIP_CORE is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      REQ_ARGS_TYPE         is record
              in_c_by_word          :  integer range 0 to QCONV_PARAM.MAX_IN_C_BY_WORD;
              in_w                  :  integer range 0 to QCONV_PARAM.MAX_IN_W;
              in_h                  :  integer range 0 to QCONV_PARAM.MAX_IN_H;
              out_c                 :  integer range 0 to QCONV_PARAM.MAX_OUT_C;
              out_w                 :  integer range 0 to QCONV_PARAM.MAX_OUT_W;
              out_h                 :  integer range 0 to QCONV_PARAM.MAX_OUT_H;
              left_pad_size         :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
              right_pad_size        :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
              top_pad_size          :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
              bottom_pad_size       :  integer range 0 to QCONV_PARAM.MAX_PAD_SIZE;
              k3x3                  :  std_logic;
              use_th                :  std_logic;
              param_in              :  std_logic;
    end record;
    constant  REQ_ARGS_NULL         :  REQ_ARGS_TYPE := (
              in_c_by_word          =>  0 ,
              in_w                  =>  0 ,
              in_h                  =>  0 ,
              out_c                 =>  0 ,
              out_w                 =>  0 ,
              out_h                 =>  0 ,
              left_pad_size         =>  0 ,
              right_pad_size        =>  0 ,
              top_pad_size          =>  0 ,
              bottom_pad_size       =>  0 ,
              k3x3                  => '0',
              use_th                => '0',
              param_in              => '0'
    );
    signal    req_args              :  REQ_ARGS_TYPE;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      PARAM_TYPE            is record
              IN_SHAPE              :  IMAGE_SHAPE_TYPE;
              OUT_SHAPE             :  IMAGE_SHAPE_TYPE;
              IN_DATA_STREAM        :  IMAGE_STREAM_PARAM_TYPE;
              K_DATA_STREAM         :  IMAGE_STREAM_PARAM_TYPE;
              THRESHOLDS_STREAM     :  IMAGE_STREAM_PARAM_TYPE;
              BIAS_STREAM           :  IMAGE_STREAM_PARAM_TYPE;
              MUL_STREAM            :  IMAGE_STREAM_PARAM_TYPE;
              ADD_STREAM            :  IMAGE_STREAM_PARAM_TYPE;
              ACC_STREAM            :  IMAGE_STREAM_PARAM_TYPE;
              PASS_TH_I_STREAM      :  IMAGE_STREAM_PARAM_TYPE;
              PASS_TH_Q_STREAM      :  IMAGE_STREAM_PARAM_TYPE;
              APPLY_TH_I_STREAM     :  IMAGE_STREAM_PARAM_TYPE;
              APPLY_TH_O_STREAM     :  IMAGE_STREAM_PARAM_TYPE;
              APPLY_TH_D_STREAM     :  IMAGE_STREAM_PARAM_TYPE;
              APPLY_TH_Q_STREAM     :  IMAGE_STREAM_PARAM_TYPE;
    end record;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  NEW_PARAM         return PARAM_TYPE is
        variable  param                     :  PARAM_TYPE;
        variable  stream_shape_in_c_by_word :  IMAGE_SHAPE_SIDE_TYPE;
        variable  stream_shape_in_c         :  IMAGE_SHAPE_SIDE_TYPE;
        variable  stream_shape_out_c        :  IMAGE_SHAPE_SIDE_TYPE;
        variable  stream_shape_x            :  IMAGE_SHAPE_SIDE_TYPE;
        variable  stream_shape_y            :  IMAGE_SHAPE_SIDE_TYPE;
        constant  pass_th_q_words           :  integer := OUT_DATA_BITS / QCONV_PARAM.NBITS_OUT_DATA;
        constant  apply_th_q_words          :  integer := OUT_DATA_BITS / QCONV_PARAM.NBITS_IN_DATA ;
    begin
        stream_shape_in_c_by_word := NEW_IMAGE_SHAPE_SIDE_CONSTANT(9*IN_C_UNROLL                           , TRUE, TRUE);
        stream_shape_in_c         := NEW_IMAGE_SHAPE_SIDE_CONSTANT(9*IN_C_UNROLL*QCONV_PARAM.NBITS_PER_WORD, TRUE, TRUE);
        stream_shape_out_c        := NEW_IMAGE_SHAPE_SIDE_CONSTANT(OUT_C_UNROLL, TRUE, TRUE);
        stream_shape_x            := NEW_IMAGE_SHAPE_SIDE_CONSTANT(1           , TRUE, TRUE);
        stream_shape_y            := NEW_IMAGE_SHAPE_SIDE_CONSTANT(1           , TRUE, TRUE);
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.IN_DATA_STREAM        := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD,
                                          C         => stream_shape_in_c_by_word,
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.K_DATA_STREAM        := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD,
                                          C         => stream_shape_in_c_by_word,
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.THRESHOLDS_STREAM    := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NUM_THRESHOLDS * QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, TRUE, TRUE),
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.BIAS_STREAM          := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, TRUE, TRUE),
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.MUL_STREAM           := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA + 1,
                                          C         => stream_shape_in_c,
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.ADD_STREAM           := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, TRUE, TRUE),
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.ACC_STREAM           := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, TRUE, TRUE),
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.PASS_TH_I_STREAM     := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => stream_shape_out_c,
                                          D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, FALSE, FALSE),
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.PASS_TH_Q_STREAM     := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(pass_th_q_words ,TRUE,TRUE),
                                          D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, FALSE, FALSE),
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.APPLY_TH_I_STREAM     := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, TRUE, TRUE),
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.APPLY_TH_O_STREAM     := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, TRUE, TRUE),
                                          D         => stream_shape_out_c,
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.APPLY_TH_D_STREAM     := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA,
                                          C         => stream_shape_out_c,
                                          D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, FALSE, FALSE),
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.APPLY_TH_Q_STREAM     := NEW_IMAGE_STREAM_PARAM(
                                          ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA,
                                          C         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(apply_th_q_words,TRUE,TRUE),
                                          D         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1, FALSE, FALSE),
                                          X         => stream_shape_x,
                                          Y         => stream_shape_y
                                      );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.IN_SHAPE  := NEW_IMAGE_SHAPE_EXTERNAL(
                               ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA,
                               C         => QCONV_PARAM.MAX_IN_C_BY_WORD,
                               X         => QCONV_PARAM.MAX_IN_W + 2,
                               Y         => QCONV_PARAM.MAX_IN_H + 2
                           );
        ---------------------------------------------------------------------------
        --
        ---------------------------------------------------------------------------
        param.OUT_SHAPE := NEW_IMAGE_SHAPE_EXTERNAL(
                               ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA,
                               C         => QCONV_PARAM.MAX_OUT_C,
                               X         => QCONV_PARAM.MAX_OUT_W,
                               Y         => QCONV_PARAM.MAX_OUT_H
                           );
        return param;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  PARAM                 :  PARAM_TYPE := NEW_PARAM;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    intake_data           :  std_logic_vector(PARAM.IN_DATA_STREAM.DATA.SIZE-1 downto 0);
    signal    intake_valid          :  std_logic;
    signal    intake_ready          :  std_logic;
    signal    intake_busy           :  std_logic;
    signal    intake_c_atrb_vec     :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.IN_DATA_STREAM.SHAPE.C.SIZE-1);
    signal    intake_d_atrb_vec     :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.IN_DATA_STREAM.SHAPE.D.SIZE-1);
    signal    intake_x_atrb_vec     :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.IN_DATA_STREAM.SHAPE.X.SIZE-1);
    signal    intake_y_atrb_vec     :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.IN_DATA_STREAM.SHAPE.Y.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    kernel_data           :  std_logic_vector(PARAM.K_DATA_STREAM.DATA.SIZE-1 downto 0);
    signal    kernel_valid          :  std_logic;
    signal    kernel_ready          :  std_logic;
    signal    kernel_busy           :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    thresholds_data       :  std_logic_vector(PARAM.THRESHOLDS_STREAM.DATA.SIZE-1 downto 0);
    signal    thresholds_valid      :  std_logic;
    signal    thresholds_ready      :  std_logic;
    signal    thresholds_busy       :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  bias_data             :  std_logic_vector(PARAM.BIAS_STREAM.DATA.SIZE-1 downto 0) := (others => '0');
    constant  bias_valid            :  std_logic := '1';
    signal    bias_ready            :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    mul_data              :  std_logic_vector(PARAM.MUL_STREAM.DATA.SIZE-1 downto 0);
    signal    mul_valid             :  std_logic;
    signal    mul_ready             :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    add_data              :  std_logic_vector(PARAM.ADD_STREAM.DATA.SIZE-1 downto 0);
    signal    add_valid             :  std_logic;
    signal    add_ready             :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    acc_data              :  std_logic_vector(PARAM.ACC_STREAM.DATA.SIZE-1 downto 0);
    signal    acc_valid             :  std_logic;
    signal    acc_ready             :  std_logic;
    signal    acc_c_atrb_vec        :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.ACC_STREAM.SHAPE.C.SIZE-1);
    signal    acc_d_atrb_vec        :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.ACC_STREAM.SHAPE.D.SIZE-1);
    signal    acc_x_atrb_vec        :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.ACC_STREAM.SHAPE.X.SIZE-1);
    signal    acc_y_atrb_vec        :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.ACC_STREAM.SHAPE.Y.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    pass_th_i_data        :  std_logic_vector(PARAM.PASS_TH_I_STREAM.DATA.SIZE-1 downto 0);
    signal    pass_th_i_valid       :  std_logic;
    signal    pass_th_i_ready       :  std_logic;
    signal    pass_th_i_done        :  std_logic;
    signal    pass_th_i_c_vec       :  IMAGE_STREAM_ATRB_VECTOR(0 to PARAM.PASS_TH_I_STREAM.SHAPE.C.SIZE-1);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    pass_th_q_data        :  std_logic_vector(PARAM.PASS_TH_Q_STREAM.DATA.SIZE-1 downto 0);
    signal    pass_th_q_elem        :  std_logic_vector(PARAM.PASS_TH_Q_STREAM.DATA.ELEM_FIELD.SIZE-1 downto 0);
    signal    pass_th_q_last        :  std_logic;
    signal    pass_th_q_valid       :  std_logic;
    signal    pass_th_q_ready       :  std_logic;
    signal    pass_th_q_busy        :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    apply_th_i_data       :  std_logic_vector(PARAM.APPLY_TH_I_STREAM.DATA.SIZE-1 downto 0);
    signal    apply_th_i_valid      :  std_logic;
    signal    apply_th_i_ready      :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    apply_th_o_data       :  std_logic_vector(PARAM.APPLY_TH_O_STREAM.DATA.SIZE-1 downto 0);
    signal    apply_th_o_valid      :  std_logic;
    signal    apply_th_o_ready      :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    apply_th_d_data       :  std_logic_vector(PARAM.APPLY_TH_D_STREAM.DATA.SIZE-1 downto 0);
    signal    apply_th_d_done       :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    apply_th_q_data       :  std_logic_vector(PARAM.APPLY_TH_Q_STREAM.DATA.SIZE-1 downto 0);
    signal    apply_th_q_elem       :  std_logic_vector(PARAM.APPLY_TH_Q_STREAM.DATA.ELEM_FIELD.SIZE-1 downto 0);
    signal    apply_th_q_last       :  std_logic;
    signal    apply_th_q_valid      :  std_logic;
    signal    apply_th_q_ready      :  std_logic;
    signal    apply_th_q_busy       :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    outlet_data           :  std_logic_vector(OUT_DATA'length-1 downto 0);
    signal    outlet_last           :  std_logic;
    signal    outlet_valid          :  std_logic;
    signal    outlet_ready          :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    i_req_valid           :  std_logic;
    signal    i_req_ready           :  std_logic;
    signal    i_res_valid           :  std_logic;
    signal    i_res_ready           :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    k_req_valid           :  std_logic;
    signal    k_req_ready           :  std_logic;
    signal    k_res_valid           :  std_logic;
    signal    k_res_ready           :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    t_req_valid           :  std_logic;
    signal    t_req_ready           :  std_logic;
    signal    t_res_valid           :  std_logic;
    signal    t_res_ready           :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      STATE_TYPE            is (IDLE_STATE, REQ_STATE, RUN_STATE, RES_STATE);
    signal    state                 :  STATE_TYPE;
begin
    -------------------------------------------------------------------------------
    -- State Machine
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                state <= IDLE_STATE;
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                state <= IDLE_STATE;
            else
                case state is
                    when IDLE_STATE =>
                        if (REQ_VALID = '1') then
                            state <= REQ_STATE;
                        else
                            state <= IDLE_STATE;
                        end if;
                    when REQ_STATE =>
                        if (intake_busy = '1' and kernel_busy = '1') and
                           ((req_args.use_th = '1' and thresholds_busy = '1') or (req_args.use_th = '0')) then
                            state <= RUN_STATE;
                        else
                            state <= REQ_STATE;
                        end if;
                    when RUN_STATE =>
                        if (outlet_valid = '1' and outlet_ready = '1' and outlet_last = '1') then
                            state <= RES_STATE;
                        else
                            state <= RUN_STATE;
                        end if;
                    when RES_STATE =>
                        if (RES_READY = '1') then
                            state <= IDLE_STATE;
                        else
                            state <= RES_STATE;
                        end if;
                    when others =>
                            state <= IDLE_STATE;
                end case;
            end if;
        end if;
    end process;
    REQ_READY <= '1' when (state = IDLE_STATE) else '0';
    RES_VALID <= '1' when (state = RES_STATE ) else '0';
    -------------------------------------------------------------------------------
    -- req_args
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                req_args <= REQ_ARGS_NULL;
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                req_args <= REQ_ARGS_NULL;
            elsif (state = IDLE_STATE and REQ_VALID = '1') then
                req_args.in_c_by_word    <= to_integer(unsigned(IN_C_BY_WORD  ));
                req_args.in_w            <= to_integer(unsigned(IN_W          ));
                req_args.in_h            <= to_integer(unsigned(IN_H          ));
                req_args.out_c           <= to_integer(unsigned(OUT_C         ));
                req_args.out_w           <= to_integer(unsigned(OUT_W         ));
                req_args.out_h           <= to_integer(unsigned(OUT_H         ));
                req_args.left_pad_size   <= to_integer(unsigned(LEFT_PAD_SIZE ));
                req_args.right_pad_size  <= to_integer(unsigned(RIGHT_PAD_SIZE));
                req_args.top_pad_size    <= to_integer(unsigned(TOP_PAD_SIZE  ));
                req_args.bottom_pad_size <= to_integer(unsigned(BOTTOM_PAD_SIZE));
                req_args.use_th          <= USE_TH;
                req_args.param_in        <= PARAM_IN;
                if (to_integer(unsigned(K_W)) = 3) and
                   (to_integer(unsigned(K_H)) = 3) then
                    req_args.k3x3     <= '1';
                else
                    req_args.k3x3     <= '0';
                end if;
            end if;
        end if;
    end process;
    -------------------------------------------------------------------------------
    -- INPUT BUFFER
    -------------------------------------------------------------------------------
    IN_DATA_BUF: QCONV_STRIP_IN_DATA_BUFFER              -- 
        generic map (                                    --
            QCONV_PARAM     => QCONV_PARAM             , -- 
            O_PARAM         => PARAM.IN_DATA_STREAM    , -- 
            I_SHAPE         => PARAM.IN_SHAPE          , -- 
            O_SHAPE         => PARAM.OUT_SHAPE         , -- 
            ELEMENT_SIZE    => IN_BUF_SIZE             , --
            IN_C_UNROLL     => IN_C_UNROLL             , --
            OUT_C_UNROLL    => OUT_C_UNROLL            , --
            ID              => 0                         -- 
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            IN_C_BY_WORD    => req_args.in_c_by_word   , -- In  :
            IN_W            => req_args.in_w           , -- In  :
            IN_H            => req_args.in_h           , -- In  :
            OUT_C           => req_args.out_c          , -- In  :
            OUT_W           => req_args.out_w          , -- In  :
            OUT_H           => req_args.out_h          , -- In  :
            LEFT_PAD_SIZE   => req_args.left_pad_size  , -- In  :
            RIGHT_PAD_SIZE  => req_args.right_pad_size , -- In  :
            TOP_PAD_SIZE    => req_args.top_pad_size   , -- In  :
            BOTTOM_PAD_SIZE => req_args.bottom_pad_size, -- In  :
            K3x3            => req_args.k3x3           , -- In  :
            REQ_VALID       => i_req_valid             , -- In  :
            REQ_READY       => i_req_ready             , -- Out :
            RES_VALID       => i_res_valid             , -- In  :
            RES_READY       => i_res_ready             , -- Out :
            I_DATA          => IN_DATA                 , -- In  :
            I_VALID         => IN_VALID                , -- In  :
            I_READY         => IN_READY                , -- Out :
            O_DATA          => intake_data             , -- Out :
            O_VALID         => intake_valid            , -- Out :
            O_READY         => intake_ready              -- In  :
        );                                               --
    process (CLK, RST) begin
        if (RST = '1') then
                intake_busy <= '0';
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                intake_busy <= '0';
            elsif (state = REQ_STATE and i_req_ready = '1') then
                intake_busy <= '1';
            elsif (i_res_valid = '1' and i_res_ready = '1') then
                intake_busy <= '0';
            end if;
        end if;
    end process;
    i_req_valid <= '1' when (state = REQ_STATE and intake_busy = '0') else '0';
    i_res_ready <= '1' when (state = RUN_STATE and intake_busy = '1') else '0';
    intake_c_atrb_vec <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.IN_DATA_STREAM, intake_data);
    intake_d_atrb_vec <= GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.IN_DATA_STREAM, intake_data);
    intake_x_atrb_vec <= GET_ATRB_X_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.IN_DATA_STREAM, intake_data);
    intake_y_atrb_vec <= GET_ATRB_Y_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.IN_DATA_STREAM, intake_data);
    -------------------------------------------------------------------------------
    -- KERNEL WEIGHT BUFFER
    -------------------------------------------------------------------------------
    K: QCONV_STRIP_K_DATA_BUFFER                         -- 
        generic map (                                    --
            QCONV_PARAM     => QCONV_PARAM             , -- 
            O_PARAM         => PARAM.K_DATA_STREAM     , -- 
            I_SHAPE         => PARAM.IN_SHAPE          , -- 
            O_SHAPE         => PARAM.OUT_SHAPE         , -- 
            ELEMENT_SIZE    => K_BUF_SIZE              , --
            IN_C_UNROLL     => IN_C_UNROLL             , --
            OUT_C_UNROLL    => OUT_C_UNROLL            , --
            QUEUE_SIZE      => 2                       , --
            ID              => 256                       -- 
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            IN_C_BY_WORD    => req_args.in_c_by_word   , -- In  :
            OUT_C           => req_args.out_c          , -- In  :
            OUT_W           => req_args.out_w          , -- In  :
            OUT_H           => req_args.out_h          , -- In  :
            K3x3            => req_args.k3x3           , -- In  :
            REQ_WRITE       => req_args.param_in       , -- In  :
            REQ_READ        => '1'                     , -- In  :
            REQ_VALID       => k_req_valid             , -- In  :
            REQ_READY       => k_req_ready             , -- Out :
            RES_VALID       => k_res_valid             , -- In  :
            RES_READY       => k_res_ready             , -- Out :
            I_DATA          => K_DATA                  , -- In  :
            I_VALID         => K_VALID                 , -- In  :
            I_READY         => K_READY                 , -- Out :
            O_DATA          => kernel_data             , -- Out :
            O_VALID         => kernel_valid            , -- Out :
            O_READY         => kernel_ready              -- In  :
        );                                               --
    process (CLK, RST) begin
        if (RST = '1') then
                kernel_busy <= '0';
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                kernel_busy <= '0';
            elsif (state = REQ_STATE and k_req_ready = '1') then
                kernel_busy <= '1';
            elsif (k_res_valid = '1' and k_res_ready = '1') then
                kernel_busy <= '0';
            end if;
        end if;
    end process;
    k_req_valid <= '1' when (state = REQ_STATE and kernel_busy = '0') else '0';
    k_res_ready <= '1' when (state = RUN_STATE   and kernel_busy = '1') else '0';
    -------------------------------------------------------------------------------
    -- THRESHOLDS BUFFER
    -------------------------------------------------------------------------------
    TH: QCONV_STRIP_TH_DATA_BUFFER                       -- 
        generic map (                                    --
            QCONV_PARAM     => QCONV_PARAM             , -- 
            O_PARAM         => PARAM.THRESHOLDS_STREAM , -- 
            O_SHAPE         => PARAM.OUT_SHAPE         , -- 
            ELEMENT_SIZE    => TH_BUF_SIZE             , --
            OUT_C_UNROLL    => OUT_C_UNROLL            , --
            ID              => 512                       -- 
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            OUT_C           => req_args.out_c          , -- In  :
            OUT_W           => req_args.out_w          , -- In  :
            OUT_H           => req_args.out_h          , -- In  :
            REQ_WRITE       => req_args.param_in       , -- In  :
            REQ_READ        => '1'                     , -- In  :
            REQ_VALID       => t_req_valid             , -- In  :
            REQ_READY       => t_req_ready             , -- Out :
            RES_VALID       => t_res_valid             , -- In  :
            RES_READY       => t_res_ready             , -- Out :
            I_DATA          => TH_DATA                 , -- In  :
            I_VALID         => TH_VALID                , -- In  :
            I_READY         => TH_READY                , -- Out :
            O_DATA          => thresholds_data         , -- Out :
            O_VALID         => thresholds_valid        , -- Out :
            O_READY         => thresholds_ready          -- In  :
        );                                               --
    process (CLK, RST) begin
        if (RST = '1') then
                thresholds_busy <= '0';
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                thresholds_busy <= '0';
            elsif (state = REQ_STATE and req_args.use_th = '1' and t_req_ready = '1') then
                thresholds_busy <= '1';
            elsif (t_res_valid = '1' and t_res_ready = '1') then
                thresholds_busy <= '0';
            end if;
        end if;
    end process;
    t_req_valid <= '1' when (state = REQ_STATE and req_args.use_th = '1' and thresholds_busy = '0') else '0';
    t_res_ready <= '1' when (state = RUN_STATE and                           thresholds_busy = '1') else '0';
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    MUL: QCONV_MULTIPLIER                                -- 
        generic map (                                    -- 
            QCONV_PARAM     => QCONV_PARAM             , --
            I_PARAM         => PARAM.IN_DATA_STREAM    , --
            K_PARAM         => PARAM.K_DATA_STREAM     , --
            O_PARAM         => PARAM.MUL_STREAM        , --
            CHECK_K_VALID   => 0                       , -- 
            QUEUE_SIZE      => 1                         --
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            I_DATA          => intake_data             , -- In  :
            I_VALID         => intake_valid            , -- In  :
            I_READY         => intake_ready            , -- Out :
            K_DATA          => kernel_data             , -- In  :
            K_VALID         => kernel_valid            , -- In  :
            K_READY         => kernel_ready            , -- Out :
            O_DATA          => mul_data                , -- Out :
            O_VALID         => mul_valid               , -- Out :
            O_READY         => mul_ready                 -- In  :
        );                                               -- 
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    ADD: CONVOLUTION_INT_ADDER_TREE                      -- 
        generic map (                                    -- 
            I_PARAM         => PARAM.MUL_STREAM        , --
            O_PARAM         => PARAM.ADD_STREAM        , --
            QUEUE_SIZE      => 1                       , --
            SIGN            => TRUE                      --
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            I_DATA          => mul_data                , -- In  :
            I_VALID         => mul_valid               , -- In  :
            I_READY         => mul_ready               , -- Out :
            O_DATA          => add_data                , -- Out :
            O_VALID         => add_valid               , -- Out :
            O_READY         => add_ready                 -- In  :
        );                                               -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    ACC: CONVOLUTION_INT_ACCUMULATOR                     -- 
        generic map (                                    -- 
            I_PARAM         => PARAM.ADD_STREAM        , --
            O_PARAM         => PARAM.ACC_STREAM        , --
            B_PARAM         => PARAM.BIAS_STREAM       , --
            QUEUE_SIZE      => 2                       , --
            SIGN            => TRUE                      --
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            I_DATA          => add_data                , -- In  :
            I_VALID         => add_valid               , -- In  :
            I_READY         => add_ready               , -- Out :
            B_DATA          => bias_data               , -- In  :
            B_VALID         => bias_valid              , -- In  :
            B_READY         => bias_ready              , -- Out :
            O_DATA          => acc_data                , -- Out :
            O_VALID         => acc_valid               , -- Out :
            O_READY         => acc_ready                 -- In  :
        );                                               --
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    acc_c_atrb_vec    <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.ACC_STREAM, acc_data);
    acc_d_atrb_vec    <= GET_ATRB_D_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.ACC_STREAM, acc_data);
    acc_x_atrb_vec    <= GET_ATRB_X_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.ACC_STREAM, acc_data);
    acc_y_atrb_vec    <= GET_ATRB_Y_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.ACC_STREAM, acc_data);
    acc_ready         <= '1' when (req_args.use_th = '1' and apply_th_i_ready = '1') or
                                  (req_args.use_th = '0' and pass_th_i_ready  = '1') else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    PASS_TH: IMAGE_STREAM_CHANNEL_REDUCER                -- 
        generic map (                                    -- 
            I_PARAM         => PARAM.PASS_TH_I_STREAM  , --
            O_PARAM         => PARAM.PASS_TH_Q_STREAM  , --
            C_SIZE          => 0                       , --
            C_DONE          => 0                         --
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            BUSY            => pass_th_q_busy          , -- Out :
            I_DATA          => pass_th_i_data          , -- In  :
            I_DONE          => pass_th_i_done          , -- In  :
            I_VALID         => pass_th_i_valid         , -- In  :
            I_READY         => pass_th_i_ready         , -- Out :
            O_DATA          => pass_th_q_data          , -- Out :
            O_VALID         => pass_th_q_valid         , -- Out :
            O_READY         => pass_th_q_ready           -- In  :
        );                                               -- 
    pass_th_i_data   <= CONVOLUTION_PIPELINE_TO_IMAGE_STREAM(PARAM.PASS_TH_I_STREAM, PARAM.ACC_STREAM, acc_data);
    pass_th_i_c_vec  <= GET_ATRB_C_VECTOR_FROM_IMAGE_STREAM_DATA(PARAM.PASS_TH_I_STREAM, pass_th_i_data);
    pass_th_i_done   <= '1' when (IMAGE_STREAM_DATA_IS_LAST_C(PARAM.PASS_TH_I_STREAM, pass_th_i_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_X(PARAM.PASS_TH_I_STREAM, pass_th_i_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_Y(PARAM.PASS_TH_I_STREAM, pass_th_i_data)) else '0';
    pass_th_i_valid  <= '1' when (req_args.use_th = '0' and acc_valid = '1') else '0';
    pass_th_q_elem   <= pass_th_q_data(PARAM.PASS_TH_Q_STREAM.DATA.ELEM_FIELD.HI downto PARAM.PASS_TH_Q_STREAM.DATA.ELEM_FIELD.LO);
    pass_th_q_last   <= '1' when (IMAGE_STREAM_DATA_IS_LAST_C(PARAM.PASS_TH_Q_STREAM, pass_th_q_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_X(PARAM.PASS_TH_Q_STREAM, pass_th_q_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_Y(PARAM.PASS_TH_Q_STREAM, pass_th_q_data)) else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    APPLY_TH: QCONV_APPLY_THRESHOLDS                     -- 
        generic map (                                    --
            QCONV_PARAM     => QCONV_PARAM             , -- 
            I_PARAM         => PARAM.APPLY_TH_I_STREAM , -- 
            T_PARAM         => PARAM.THRESHOLDS_STREAM , --
            O_PARAM         => PARAM.APPLY_TH_O_STREAM , --
            QUEUE_SIZE      => 2                         --
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            I_DATA          => apply_th_i_data         , -- In  :
            I_VALID         => apply_th_i_valid        , -- In  :
            I_READY         => apply_th_i_ready        , -- Out :
            T_DATA          => thresholds_data         , -- In  :
            T_VALID         => thresholds_valid        , -- In  :
            T_READY         => thresholds_ready        , -- Out :
            O_DATA          => apply_th_o_data         , -- Out :
            O_VALID         => apply_th_o_valid        , -- Out :
            O_READY         => apply_th_o_ready          -- In  :
        );                                               -- 
    apply_th_i_data  <= acc_data;
    apply_th_i_valid <= '1' when (req_args.use_th = '1' and acc_valid = '1') else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    APPLY_QUEUE: IMAGE_STREAM_CHANNEL_REDUCER            -- 
        generic map (                                    -- 
            I_PARAM         => PARAM.APPLY_TH_D_STREAM , --
            O_PARAM         => PARAM.APPLY_TH_Q_STREAM , --
            C_SIZE          => 0                       , --
            C_DONE          => 0                         --
        )                                                -- 
        port map (                                       -- 
            CLK             => CLK                     , -- In  :
            RST             => RST                     , -- In  :
            CLR             => CLR                     , -- In  :
            BUSY            => apply_th_q_busy         , -- Out :
            I_DATA          => apply_th_d_data         , -- In  :
            I_DONE          => apply_th_d_done         , -- In  :
            I_VALID         => apply_th_o_valid        , -- In  :
            I_READY         => apply_th_o_ready        , -- Out :
            O_DATA          => apply_th_q_data         , -- Out :
            O_VALID         => apply_th_q_valid        , -- Out :
            O_READY         => apply_th_q_ready          -- In  :
        );                                               --
    apply_th_d_data  <= CONVOLUTION_PIPELINE_TO_IMAGE_STREAM(PARAM.APPLY_TH_D_STREAM, PARAM.APPLY_TH_O_STREAM, apply_th_o_data);
    apply_th_d_done  <= '1' when (IMAGE_STREAM_DATA_IS_LAST_C(PARAM.APPLY_TH_D_STREAM, apply_th_d_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_X(PARAM.APPLY_TH_D_STREAM, apply_th_d_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_Y(PARAM.APPLY_TH_D_STREAM, apply_th_d_data)) else '0';
    apply_th_q_last  <= '1' when (IMAGE_STREAM_DATA_IS_LAST_C(PARAM.APPLY_TH_Q_STREAM, apply_th_q_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_X(PARAM.APPLY_TH_Q_STREAM, apply_th_q_data) and
                                  IMAGE_STREAM_DATA_IS_LAST_Y(PARAM.APPLY_TH_Q_STREAM, apply_th_q_data)) else '0';
    process (apply_th_q_data)
        variable  elem_data     :  std_logic_vector(PARAM.APPLY_TH_Q_STREAM.DATA.ELEM_FIELD.SIZE-1 downto 0);
        constant  OUT_WORD_BITS :  integer := QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD;
        constant  OUT_WORDS     :  integer := OUT_DATA_BITS / OUT_WORD_BITS;
    begin
        elem_data := apply_th_q_data(PARAM.APPLY_TH_Q_STREAM.DATA.ELEM_FIELD.HI downto PARAM.APPLY_TH_Q_STREAM.DATA.ELEM_FIELD.LO);
        for out_pos  in 0 to OUT_WORDS-1 loop
        for word_pos in 0 to QCONV_PARAM.NBITS_PER_WORD-1 loop
        for data_pos in 0 to QCONV_PARAM.NBITS_IN_DATA -1 loop
            apply_th_q_elem(out_pos*OUT_WORD_BITS + data_pos*QCONV_PARAM.NBITS_PER_WORD + word_pos) <= elem_data(out_pos*OUT_WORD_BITS + word_pos*QCONV_PARAM.NBITS_IN_DATA + data_pos);
        end loop;
        end loop;
        end loop;
    end process;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    outlet_data  <= apply_th_q_elem when (req_args.use_th = '1') else pass_th_q_elem;
    outlet_valid <= '1' when (req_args.use_th = '1' and apply_th_q_valid = '1') or
                             (req_args.use_th = '0' and pass_th_q_valid  = '1') else '0';
    outlet_last  <= '1' when (req_args.use_th = '1' and apply_th_q_last  = '1') or
                             (req_args.use_th = '0' and pass_th_q_last   = '1') else '0';
    apply_th_q_ready <= '1' when (req_args.use_th = '1' and outlet_ready = '1') else '0';
    pass_th_q_ready  <= '1' when (req_args.use_th = '0' and outlet_ready = '1') else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    OUT_DATA     <= outlet_data;
    OUT_VALID    <= outlet_valid;
    OUT_LAST     <= outlet_last;
    outlet_ready <= OUT_READY;
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_in_data_axi_reader.vhd
--!     @brief   Quantized Convolution (strip) In Data AXI Reader Module
--!     @version 0.1.0
--!     @date    2019/4/26
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_IN_DATA_AXI_READER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 12;
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_ARID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_ARADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_ARLEN       : out std_logic_vector(7 downto 0);
        AXI_ARSIZE      : out std_logic_vector(2 downto 0);
        AXI_ARBURST     : out std_logic_vector(1 downto 0);
        AXI_ARLOCK      : out std_logic_vector(0 downto 0);
        AXI_ARCACHE     : out std_logic_vector(3 downto 0);
        AXI_ARPROT      : out std_logic_vector(2 downto 0);
        AXI_ARQOS       : out std_logic_vector(3 downto 0);
        AXI_ARREGION    : out std_logic_vector(3 downto 0);
        AXI_ARUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_ARVALID     : out std_logic;
        AXI_ARREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_RID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_RDATA       : in  std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_RRESP       : in  std_logic_vector(1 downto 0);
        AXI_RLAST       : in  std_logic;
        AXI_RVALID      : in  std_logic;
        AXI_RREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Master Interface.
    -------------------------------------------------------------------------------
        O_DATA          : out std_logic_vector(QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD-1 downto 0);
        O_LAST          : out std_logic;
        O_VALID         : out std_logic;
        O_READY         : in  std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_IN_C        : in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        REQ_IN_W        : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        REQ_IN_H        : in  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        REQ_X_POS       : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        REQ_X_SIZE      : in  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end QCONV_STRIP_IN_DATA_AXI_READER;
-----------------------------------------------------------------------------------
-- アーキテクチャ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.AXI4_TYPES.all;
use     PIPEWORK.AXI4_COMPONENTS.AXI4_MASTER_READ_INTERFACE;
use     PIPEWORK.PUMP_COMPONENTS.PUMP_STREAM_INTAKE_CONTROLLER;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_SLICE_MASTER_CONTROLLER;
use     PIPEWORK.COMPONENTS.SDPRAM;
architecture RTL of QCONV_STRIP_IN_DATA_AXI_READER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MAX(A,B: integer) return integer is
    begin
        if (A > B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B: integer) return integer is
    begin
        if (A < B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B,C: integer) return integer is
    begin
        return MIN(A,MIN(B,C));
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function CALC_BITS(SIZE:integer) return integer is
        variable bits : integer;
    begin
        bits := 0;
        while (2**bits < SIZE) loop
            bits := bits + 1;
        end loop;
        return bits;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  IMAGE_SHAPE           :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                           ELEM_BITS => QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD,
                                           C         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_IN_C_BY_WORD),
                                           X         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_IN_W),
                                           Y         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_IN_H)
                                       );
    signal    req_image_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_image_x_size      :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_image_y_size      :  integer range 0 to IMAGE_SHAPE.Y.MAX_SIZE;
    signal    req_slice_x_pos       :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_slice_x_size      :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_axi_addr          :  std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    -------------------------------------------------------------------------------
    -- 一回のトランザクションで転送する最大転送バイト数
    -------------------------------------------------------------------------------
    constant  MAX_XFER_BYTES        :  integer := MIN(4096, 256*(AXI_DATA_WIDTH/8), 2**AXI_XFER_SIZE);
    constant  MAX_XFER_SIZE         :  integer := CALC_BITS(MAX_XFER_BYTES);
    ------------------------------------------------------------------------------
    -- バッファの容量をバイト数で示す.
    ------------------------------------------------------------------------------
    constant  BUF_BYTES             :  integer := MAX_XFER_BYTES*2;
    ------------------------------------------------------------------------------
    -- バッファの容量(バイト数)を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DEPTH             :  integer := CALC_BITS(BUF_BYTES);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を示す.
    ------------------------------------------------------------------------------
    constant  BUF_WIDTH             :  integer := MAX(AXI_DATA_WIDTH, O_DATA'length);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DATA_BIT_SIZE     :  integer := CALC_BITS(BUF_WIDTH);
    ------------------------------------------------------------------------------
    -- 入力側のフロー制御用定数.
    ------------------------------------------------------------------------------
    constant  I_FLOW_VALID          :  integer := 1;
    constant  I_USE_PUSH_BUF_SIZE   :  integer := 0;
    constant  I_FIXED_FLOW_OPEN     :  integer := 0;
    constant  I_FIXED_POOL_OPEN     :  integer := 1;
    constant  I_REQ_ADDR_VALID      :  integer := 1;
    constant  I_REQ_SIZE_VALID      :  integer := 1;
    constant  I_FLOW_READY_LEVEL    :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(BUF_BYTES - MAX_XFER_BYTES    , BUF_DEPTH+1));
    constant  I_BUF_READY_LEVEL     :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(BUF_BYTES - 1*AXI_DATA_WIDTH/8, BUF_DEPTH+1));
    constant  I_MAX_REQ_SIZE        :  integer := IMAGE_SHAPE.X.MAX_SIZE * IMAGE_SHAPE.C.MAX_SIZE * IMAGE_SHAPE.ELEM_BITS / 8;
    constant  REQ_SIZE_WIDTH        :  integer := CALC_BITS(I_MAX_REQ_SIZE+1);
    -------------------------------------------------------------------------------
    -- AXI I/F 定数
    -------------------------------------------------------------------------------
    constant  AXI_REQ_PROT          :  AXI4_APROT_TYPE
                                    := std_logic_vector(to_unsigned(AXI_PROT  , AXI4_APROT_WIDTH  ));
    constant  AXI_REQ_QOS           :  AXI4_AQOS_TYPE
                                    := std_logic_vector(to_unsigned(AXI_QOS   , AXI4_AQOS_WIDTH   ));
    constant  AXI_REQ_REGION        :  AXI4_AREGION_TYPE
                                    := std_logic_vector(to_unsigned(AXI_REGION, AXI4_AREGION_WIDTH));
    constant  AXI_REQ_CACHE         :  AXI4_ACACHE_TYPE
                                    := std_logic_vector(to_unsigned(AXI_CACHE , AXI4_ACACHE_WIDTH ));
    constant  AXI_REQ_ID            :  std_logic_vector(AXI_ID_WIDTH -1 downto 0)
                                    := std_logic_vector(to_unsigned(AXI_ID    , AXI_ID_WIDTH      ));
    constant  AXI_REQ_LOCK          :  AXI4_ALOCK_TYPE  := (others => '0');
    constant  AXI_REQ_SPECULATIVE   :  std_logic := '1';
    constant  AXI_REQ_SAFETY        :  std_logic := '0';
    constant  AXI_ALIGNMENT_BITS    :  integer := 32;
    constant  AXI_ACK_REGS          :  integer := 1;
    constant  AXI_RDATA_REGS        :  integer := 3;
    constant  OPEN_INFO_BITS        :  integer := 4;
    constant  CLOSE_INFO_BITS       :  integer := 4;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_tran_start          :  std_logic;
    signal    i_tran_first          :  std_logic;
    signal    i_tran_last           :  std_logic;
    signal    i_tran_addr           :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_tran_addr_load      :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_tran_size           :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_tran_size_load      :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_tran_busy           :  std_logic;
    signal    i_tran_done           :  std_logic;
    signal    i_tran_error          :  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_req_valid           :  std_logic;
    signal    i_req_addr            :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_req_size            :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_req_buf_ptr         :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    i_req_first           :  std_logic;
    signal    i_req_last            :  std_logic;
    signal    i_req_ready           :  std_logic;
    signal    i_ack_valid           :  std_logic;
    signal    i_ack_size            :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_ack_error           :  std_logic;
    signal    i_ack_next            :  std_logic;
    signal    i_ack_last            :  std_logic;
    signal    i_ack_stop            :  std_logic;
    signal    i_ack_none            :  std_logic;
    signal    i_xfer_busy           :  std_logic;
    signal    i_xfer_done           :  std_logic;
    signal    i_xfer_error          :  std_logic;
    signal    i_flow_ready          :  std_logic;
    signal    i_flow_pause          :  std_logic;
    signal    i_flow_stop           :  std_logic;
    signal    i_flow_last           :  std_logic;
    signal    i_flow_size           :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_fin_valid      :  std_logic;
    signal    i_push_fin_last       :  std_logic;
    signal    i_push_fin_error      :  std_logic;
    signal    i_push_fin_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_rsv_valid      :  std_logic;
    signal    i_push_rsv_last       :  std_logic;
    signal    i_push_rsv_error      :  std_logic;
    signal    i_push_rsv_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_buf_reset      :  std_logic;
    signal    i_push_buf_valid      :  std_logic;
    signal    i_push_buf_last       :  std_logic;
    signal    i_push_buf_error      :  std_logic;
    signal    i_push_buf_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_buf_ready      :  std_logic;
    signal    i_open                :  std_logic;
    constant  i_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0) := (others => '0');
    constant  i_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0) := (others => '0');
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    buf_ren               :  std_logic;
    signal    buf_rptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_rdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_wen               :  std_logic;
    signal    buf_wptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_wdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_we                :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    signal    buf_ben               :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    o_open                :  std_logic;
    signal    o_done                :  std_logic;
    signal    o_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0);
    signal    o_open_valid          :  std_logic;
    signal    o_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0);
    signal    o_close_valid         :  std_logic;
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    req_image_c_size <= to_integer(to_01(unsigned(REQ_IN_C  )));
    req_image_x_size <= to_integer(to_01(unsigned(REQ_IN_W  )));
    req_image_y_size <= to_integer(to_01(unsigned(REQ_IN_H  )));
    req_slice_x_pos  <= to_integer(to_01(unsigned(REQ_X_POS )));
    req_slice_x_size <= to_integer(to_01(unsigned(REQ_X_SIZE)));
    req_axi_addr     <= std_logic_vector(resize(unsigned(REQ_ADDR), AXI_ADDR_WIDTH));
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    MST_CTRL: IMAGE_SLICE_MASTER_CONTROLLER              -- 
        generic map (                                    -- 
            SOURCE_SHAPE        => IMAGE_SHAPE         , --
            SLICE_SHAPE         => IMAGE_SHAPE         , --
            MAX_SLICE_C_POS     => 0                   , --
            MAX_SLICE_X_POS     => IMAGE_SHAPE.X.MAX_SIZE , --
            MAX_SLICE_Y_POS     => 0                   , --
            ADDR_BITS           => AXI_ADDR_WIDTH      , --
            SIZE_BITS           => REQ_SIZE_WIDTH        --
        )                                                -- 
        port map (                                       -- 
        -------------------------------------------------------------------------------
        -- クロック&リセット信号
        -------------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            SOURCE_C_SIZE       => req_image_c_size    , -- In  :
            SOURCE_X_SIZE       => req_image_x_size    , -- In  :
            SOURCE_Y_SIZE       => req_image_y_size    , -- In  :
            SLICE_C_SIZE        => req_image_c_size    , -- In  :
            SLICE_X_POS         => req_slice_x_pos     , -- In  :
            SLICE_X_SIZE        => req_slice_x_size    , -- In  :
            SLICE_Y_SIZE        => req_image_y_size    , -- In  :
            REQ_ADDR            => req_axi_addr        , -- In  :
            REQ_VALID           => REQ_VALID           , -- In  :
            REQ_READY           => REQ_READY           , -- Out :
            RES_NONE            => RES_NONE            , -- Out :
            RES_ERROR           => RES_ERROR           , -- Out :
            RES_VALID           => RES_VALID           , -- Out :
            RES_READY           => RES_READY           , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            MST_ADDR            => i_tran_addr         , -- Out :
            MST_SIZE            => i_tran_size         , -- Out :
            MST_FIRST           => i_tran_first        , -- Out :
            MST_LAST            => i_tran_last         , -- Out :
            MST_START           => i_tran_start        , -- Out :
            MST_BUSY            => i_tran_busy         , -- In  :
            MST_DONE            => i_tran_done         , -- In  :
            MST_ERROR           => i_tran_error          -- In  :
        );                                               -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    i_tran_addr_load <= (others => '1') when (i_tran_start = '1') else (others => '0');
    i_tran_size_load <= (others => '1') when (i_tran_start = '1') else (others => '0');
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    PUMP_CTRL: PUMP_STREAM_INTAKE_CONTROLLER             -- 
        generic map (                                    -- 
            I_CLK_RATE          => 1                   , --
            I_REQ_ADDR_VALID    => I_REQ_ADDR_VALID    , --
            I_REQ_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            I_REG_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            I_REQ_SIZE_VALID    => I_REQ_SIZE_VALID    , --
            I_REQ_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            I_REG_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            I_REG_MODE_BITS     => 1                   , --
            I_REG_STAT_BITS     => 1                   , --
            I_USE_PUSH_BUF_SIZE => I_USE_PUSH_BUF_SIZE , --
            I_FIXED_FLOW_OPEN   => I_FIXED_FLOW_OPEN   , --
            I_FIXED_POOL_OPEN   => I_FIXED_POOL_OPEN   , --
            O_CLK_RATE          => 1                   , --
            O_DATA_BITS         => O_DATA'length       , --
            BUF_DEPTH           => BUF_DEPTH           , --
            BUF_DATA_BITS       => BUF_WIDTH           , --
            I2O_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            I2O_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            O2I_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            O2I_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            I2O_DELAY_CYCLE     => 1                     --
        )                                                -- 
        port map (                                       -- 
        ---------------------------------------------------------------------------
        --Reset Signals.
        ---------------------------------------------------------------------------
            RST                 => RST                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Clock and Clock Enable.
        ---------------------------------------------------------------------------
            I_CLK               => CLK                 , --  In  :
            I_CLR               => CLR                 , --  In  :
            I_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Control Register Interface.
        ---------------------------------------------------------------------------
            I_ADDR_L            => i_tran_addr_load    , --  In  :
            I_ADDR_D            => i_tran_addr         , --  In  :
            I_SIZE_L            => i_tran_size_load    , --  In  :
            I_SIZE_D            => i_tran_size         , --  In  :
            I_START_L           => i_tran_start        , --  In  :
            I_START_D           => i_tran_start        , --  In  :
            I_FIRST_L           => i_tran_start        , --  In  :
            I_FIRST_D           => i_tran_first        , --  In  :
            I_LAST_L            => i_tran_start        , --  In  :
            I_LAST_D            => i_tran_last         , --  In  :
            I_DONE_EN_L         => i_tran_start        , --  In  :
            I_DONE_EN_D         => '0'                 , --  In  :
            I_DONE_ST_L         => i_tran_start        , --  In  :
            I_DONE_ST_D         => '0'                 , --  In  :
            I_ERR_ST_L          => i_tran_start        , --  In  :
            I_ERR_ST_D          => '0'                 , --  In  :
            I_CLOSE_ST_L        => i_tran_start        , --  In  :
            I_CLOSE_ST_D        => '0'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Configuration Signals.
        ---------------------------------------------------------------------------
            I_BUF_READY_LEVEL   => I_BUF_READY_LEVEL   , --  In  :
            I_FLOW_READY_LEVEL  => I_FLOW_READY_LEVEL  , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transaction Command Request Signals.
        ---------------------------------------------------------------------------
            I_REQ_VALID         => i_req_valid         , --  Out :
            I_REQ_ADDR          => i_req_addr          , --  Out :
            I_REQ_SIZE          => i_req_size          , --  Out :
            I_REQ_BUF_PTR       => i_req_buf_ptr       , --  Out :
            I_REQ_FIRST         => i_req_first         , --  Out :
            I_REQ_LAST          => i_req_last          , --  Out :
            I_REQ_READY         => i_req_ready         , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transaction Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            I_ACK_VALID         => i_ack_valid         , --  In  :
            I_ACK_SIZE          => i_ack_size          , --  In  :
            I_ACK_ERROR         => i_ack_error         , --  In  :
            I_ACK_NEXT          => i_ack_next          , --  In  :
            I_ACK_LAST          => i_ack_last          , --  In  :
            I_ACK_STOP          => i_ack_stop          , --  In  :
            I_ACK_NONE          => i_ack_none          , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transfer Status Signals.
        ---------------------------------------------------------------------------
            I_XFER_BUSY         => i_xfer_busy         , --  In  :
            I_XFER_DONE         => i_xfer_done         , --  In  :
            I_XFER_ERROR        => i_xfer_error        , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Flow Control Signals.
        ---------------------------------------------------------------------------
            I_FLOW_READY        => i_flow_ready        , --  Out :
            I_FLOW_PAUSE        => i_flow_pause        , --  Out :
            I_FLOW_STOP         => i_flow_stop         , --  Out :
            I_FLOW_LAST         => i_flow_last         , --  Out :
            I_FLOW_SIZE         => i_flow_size         , --  Out :
            I_PUSH_FIN_VALID    => i_push_fin_valid    , --  In  :
            I_PUSH_FIN_LAST     => i_push_fin_last     , --  In  :
            I_PUSH_FIN_ERROR    => i_push_fin_error    , --  In  :
            I_PUSH_FIN_SIZE     => i_push_fin_size     , --  In  :
            I_PUSH_RSV_VALID    => i_push_rsv_valid    , --  In  :
            I_PUSH_RSV_LAST     => i_push_rsv_last     , --  In  :
            I_PUSH_RSV_ERROR    => i_push_rsv_error    , --  In  :
            I_PUSH_RSV_SIZE     => i_push_rsv_size     , --  In  :
            I_PUSH_BUF_RESET    => i_push_buf_reset    , --  In  :
            I_PUSH_BUF_VALID    => i_push_buf_valid    , --  In  :
            I_PUSH_BUF_LAST     => i_push_buf_last     , --  In  :
            I_PUSH_BUF_ERROR    => i_push_buf_error    , --  In  :
            I_PUSH_BUF_SIZE     => i_push_buf_size     , --  In  :
            I_PUSH_BUF_READY    => i_push_buf_ready    , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Status.
        ---------------------------------------------------------------------------
            I_OPEN              => i_open              , --  Out :
            I_TRAN_BUSY         => i_tran_busy         , --  Out :
            I_TRAN_DONE         => i_tran_done         , --  Out :
            I_TRAN_ERROR        => i_tran_error        , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            I_I2O_OPEN_INFO     => i_open_info         , --  In  :
            I_I2O_CLOSE_INFO    => i_close_info        , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Clock and Clock Enable.
        ---------------------------------------------------------------------------
            O_CLK               => CLK                 , --  In  :
            O_CLR               => CLR                 , --  In  :
            O_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Stream Interface.
        ---------------------------------------------------------------------------
            O_DATA              => O_DATA              , --  Out :
            O_STRB              => open                , --  Out :
            O_LAST              => O_LAST              , --  Out :
            O_VALID             => O_VALID             , --  Out :
            O_READY             => O_READY             , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            O_O2I_OPEN_INFO     => o_open_info         , --  In  :
            O_O2I_OPEN_VALID    => o_open_valid        , --  In  :
            O_O2I_CLOSE_INFO    => o_close_info        , --  In  :
            O_O2I_CLOSE_VALID   => o_close_valid       , --  In  :
            O_I2O_OPEN_INFO     => o_open_info         , --  Out :
            O_I2O_OPEN_VALID    => o_open_valid        , --  Out :
            O_I2O_CLOSE_INFO    => o_close_info        , --  Out :
            O_I2O_CLOSE_VALID   => o_close_valid       , --  Out :
        ---------------------------------------------------------------------------
        -- Outlet Buffer Read Interface.
        ---------------------------------------------------------------------------
            BUF_REN             => buf_ren             , --  Out :
            BUF_PTR             => buf_rptr            , --  Out :
            BUF_DATA            => buf_rdata             --  In  :
        );                                               --
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    AXI_IF: AXI4_MASTER_READ_INTERFACE                   -- 
        generic map (                                    -- 
            AXI4_ADDR_WIDTH     => AXI_ADDR_WIDTH      , -- 
            AXI4_DATA_WIDTH     => AXI_DATA_WIDTH      , --   
            AXI4_ID_WIDTH       => AXI_ID_WIDTH        , --   
            VAL_BITS            => 1                   , --   
            REQ_SIZE_BITS       => REQ_SIZE_WIDTH      , --   
            REQ_SIZE_VALID      => 1                   , --   
            FLOW_VALID          => I_FLOW_VALID        , --   
            BUF_DATA_WIDTH      => BUF_WIDTH           , --   
            BUF_PTR_BITS        => BUF_DEPTH           , --   
            ALIGNMENT_BITS      => AXI_ALIGNMENT_BITS  , --   
            XFER_SIZE_BITS      => BUF_DEPTH+1         , --   
            XFER_MIN_SIZE       => MAX_XFER_SIZE       , --   
            XFER_MAX_SIZE       => MAX_XFER_SIZE       , --   
            QUEUE_SIZE          => AXI_REQ_QUEUE       , --   
            RDATA_REGS          => AXI_RDATA_REGS      , --   
            ACK_REGS            => AXI_ACK_REGS          --   
        )                                                -- 
        port map(                                        --
        ---------------------------------------------------------------------------
        -- Clock and Reset Signals.
        ---------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            ARID                => AXI_ARID            , -- Out :
            ARADDR              => AXI_ARADDR          , -- Out :
            ARLEN               => AXI_ARLEN           , -- Out :
            ARSIZE              => AXI_ARSIZE          , -- Out :
            ARBURST             => AXI_ARBURST         , -- Out :
            ARLOCK              => AXI_ARLOCK          , -- Out :
            ARCACHE             => AXI_ARCACHE         , -- Out :
            ARPROT              => AXI_ARPROT          , -- Out :
            ARQOS               => AXI_ARQOS           , -- Out :
            ARREGION            => AXI_ARREGION        , -- Out :
            ARVALID             => AXI_ARVALID         , -- Out :
            ARREADY             => AXI_ARREADY         , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            RID                 => AXI_RID             , -- In  :
            RDATA               => AXI_RDATA           , -- In  :
            RRESP               => AXI_RRESP           , -- In  :
            RLAST               => AXI_RLAST           , -- In  :
            RVALID              => AXI_RVALID          , -- In  :
            RREADY              => AXI_RREADY          , -- Out :
        ---------------------------------------------------------------------------
        -- Command Request Signals.
        ---------------------------------------------------------------------------
            XFER_SIZE_SEL       => "1"                 , -- In  :
            REQ_ADDR            => i_req_addr          , -- In  :
            REQ_SIZE            => i_req_size          , -- In  :
            REQ_ID              => AXI_REQ_ID          , -- In  :
            REQ_BURST           => AXI4_ABURST_INCR    , -- In  :
            REQ_LOCK            => AXI_REQ_LOCK        , -- In  :
            REQ_CACHE           => AXI_REQ_CACHE       , -- In  :
            REQ_PROT            => AXI_REQ_PROT        , -- In  :
            REQ_QOS             => AXI_REQ_QOS         , -- In  :
            REQ_REGION          => AXI_REQ_REGION      , -- In  :
            REQ_BUF_PTR         => i_req_buf_ptr       , -- In  :
            REQ_FIRST           => i_req_first         , -- In  :
            REQ_LAST            => i_req_last          , -- In  :
            REQ_SPECULATIVE     => AXI_REQ_SPECULATIVE , -- In  :
            REQ_SAFETY          => AXI_REQ_SAFETY      , -- In  :
            REQ_VAL(0)          => i_req_valid         , -- In  :
            REQ_RDY             => i_req_ready         , -- Out :
        ---------------------------------------------------------------------------
        -- Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            ACK_VAL(0)          => i_ack_valid         , -- Out :
            ACK_NEXT            => i_ack_next          , -- Out :
            ACK_LAST            => i_ack_last          , -- Out :
            ACK_ERROR           => i_ack_error         , -- Out :
            ACK_STOP            => i_ack_stop          , -- Out :
            ACK_NONE            => i_ack_none          , -- Out :
            ACK_SIZE            => i_ack_size          , -- Out :
        ---------------------------------------------------------------------------
        -- Transfer Status Signal.
        ---------------------------------------------------------------------------
            XFER_BUSY(0)        => i_xfer_busy         , -- Out :
            XFER_ERROR(0)       => i_xfer_error        , -- Out :
            XFER_DONE(0)        => i_xfer_done         , -- Out :
        ---------------------------------------------------------------------------
        -- Flow Control Signals.
        ---------------------------------------------------------------------------
            FLOW_STOP           => i_flow_stop         , -- In  :
            FLOW_PAUSE          => i_flow_pause        , -- In  :
            FLOW_LAST           => i_flow_last         , -- In  :
            FLOW_SIZE           => i_flow_size         , -- In  :
        ---------------------------------------------------------------------------
        -- Push Reserve Size Signals.
        ---------------------------------------------------------------------------
            PUSH_RSV_VAL(0)     => i_push_rsv_valid    , -- Out :
            PUSH_RSV_LAST       => i_push_rsv_last     , -- Out :
            PUSH_RSV_ERROR      => i_push_rsv_error    , -- Out :
            PUSH_RSV_SIZE       => i_push_rsv_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Push Final Size Signals.
        ---------------------------------------------------------------------------
            PUSH_FIN_VAL(0)     => i_push_fin_valid    , -- Out :
            PUSH_FIN_LAST       => i_push_fin_last     , -- Out :
            PUSH_FIN_ERROR      => i_push_fin_error    , -- Out :
            PUSH_FIN_SIZE       => i_push_fin_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Push Buffer Size Signals.
        ---------------------------------------------------------------------------
            PUSH_BUF_RESET(0)   => i_push_buf_reset    , -- Out :
            PUSH_BUF_VAL(0)     => i_push_buf_valid    , -- Out :
            PUSH_BUF_LAST       => i_push_buf_last     , -- Out :
            PUSH_BUF_ERROR      => i_push_buf_error    , -- Out :
            PUSH_BUF_SIZE       => i_push_buf_size     , -- Out :
            PUSH_BUF_RDY(0)     => i_push_buf_ready    , -- In  :
        ---------------------------------------------------------------------------
        -- Read Buffer Interface Signals.
        ---------------------------------------------------------------------------
            BUF_WEN(0)          => buf_wen             , -- Out :
            BUF_BEN             => buf_ben             , -- Out :
            BUF_DATA            => buf_wdata           , -- Out :
            BUF_PTR             => buf_wptr              -- Out :
        );                                               -- 
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    RAM: SDPRAM 
        generic map(
            DEPTH       => BUF_DEPTH+3         ,
            RWIDTH      => BUF_DATA_BIT_SIZE   , --
            WWIDTH      => BUF_DATA_BIT_SIZE   , --
            WEBIT       => BUF_DATA_BIT_SIZE-3 , --
            ID          => 0                     -- 
        )                                        -- 
        port map (                               -- 
            WCLK        => CLK                 , -- In  :
            WE          => buf_we              , -- In  :
            WADDR       => buf_wptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            WDATA       => buf_wdata           , -- In  :
            RCLK        => CLK                 , -- In  :
            RADDR       => buf_rptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            RDATA       => buf_rdata             -- Out :
        );
    buf_we <= buf_ben when (buf_wen = '1') else (others => '0');
end RTL;

-----------------------------------------------------------------------------------
--!     @file    qconv_strip_k_data_axi_reader.vhd
--!     @brief   Quantized Convolution (strip) Kernel Weight Data AXI Reader Module
--!     @version 0.1.0
--!     @date    2019/4/26
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_K_DATA_AXI_READER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 128*(64/8);
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_ARID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_ARADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_ARLEN       : out std_logic_vector(7 downto 0);
        AXI_ARSIZE      : out std_logic_vector(2 downto 0);
        AXI_ARBURST     : out std_logic_vector(1 downto 0);
        AXI_ARLOCK      : out std_logic_vector(0 downto 0);
        AXI_ARCACHE     : out std_logic_vector(3 downto 0);
        AXI_ARPROT      : out std_logic_vector(2 downto 0);
        AXI_ARQOS       : out std_logic_vector(3 downto 0);
        AXI_ARREGION    : out std_logic_vector(3 downto 0);
        AXI_ARUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_ARVALID     : out std_logic;
        AXI_ARREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_RID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_RDATA       : in  std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_RRESP       : in  std_logic_vector(1 downto 0);
        AXI_RLAST       : in  std_logic;
        AXI_RVALID      : in  std_logic;
        AXI_RREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Master Interface.
    -------------------------------------------------------------------------------
        O_DATA          : out std_logic_vector(QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD -1 downto 0);
        O_LAST          : out std_logic;
        O_VALID         : out std_logic;
        O_READY         : in  std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_IN_C        : in  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        REQ_OUT_C       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        REQ_OUT_C_POS   : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        REQ_OUT_C_SIZE  : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        REQ_K3x3        : in  std_logic;
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end QCONV_STRIP_K_DATA_AXI_READER;
-----------------------------------------------------------------------------------
-- アーキテクチャ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.AXI4_TYPES.all;
use     PIPEWORK.AXI4_COMPONENTS.AXI4_MASTER_READ_INTERFACE;
use     PIPEWORK.PUMP_COMPONENTS.PUMP_STREAM_INTAKE_CONTROLLER;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_SLICE_MASTER_CONTROLLER;
use     PIPEWORK.COMPONENTS.SDPRAM;
architecture RTL of QCONV_STRIP_K_DATA_AXI_READER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MAX(A,B: integer) return integer is
    begin
        if (A > B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B: integer) return integer is
    begin
        if (A < B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B,C: integer) return integer is
    begin
        return MIN(A,MIN(B,C));
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function CALC_BITS(SIZE:integer) return integer is
        variable bits : integer;
    begin
        bits := 0;
        while (2**bits < SIZE) loop
            bits := bits + 1;
        end loop;
        return bits;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  IMAGE_SHAPE           :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                           ELEM_BITS => QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD * 9,
                                           C         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_IN_C_BY_WORD),
                                           X         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_OUT_C),
                                           Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                                       );
    signal    req_image_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_image_x_size      :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_slice_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_slice_x_pos       :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_slice_x_size      :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_elem_bytes        :  integer range 0 to IMAGE_SHAPE.ELEM_BITS/8;
    signal    req_axi_addr          :  std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    -------------------------------------------------------------------------------
    -- 一回のトランザクションで転送する最大転送バイト数
    -------------------------------------------------------------------------------
    constant  MAX_XFER_BYTES        :  integer := MIN(4096, 256*(AXI_DATA_WIDTH/8), 2**AXI_XFER_SIZE);
    constant  MAX_XFER_SIZE         :  integer := CALC_BITS(MAX_XFER_BYTES);
    ------------------------------------------------------------------------------
    -- バッファの容量をバイト数で示す.
    ------------------------------------------------------------------------------
    constant  BUF_BYTES             :  integer := MAX_XFER_BYTES*2;
    ------------------------------------------------------------------------------
    -- バッファの容量(バイト数)を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DEPTH             :  integer := CALC_BITS(BUF_BYTES);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を示す.
    ------------------------------------------------------------------------------
    constant  BUF_WIDTH             :  integer := MAX(AXI_DATA_WIDTH, O_DATA'length);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DATA_BIT_SIZE     :  integer := CALC_BITS(BUF_WIDTH);
    ------------------------------------------------------------------------------
    -- 入力側のフロー制御用定数.
    ------------------------------------------------------------------------------
    constant  I_FLOW_VALID          :  integer := 1;
    constant  I_USE_PUSH_BUF_SIZE   :  integer := 0;
    constant  I_FIXED_FLOW_OPEN     :  integer := 0;
    constant  I_FIXED_POOL_OPEN     :  integer := 1;
    constant  I_REQ_ADDR_VALID      :  integer := 1;
    constant  I_REQ_SIZE_VALID      :  integer := 1;
    constant  I_FLOW_READY_LEVEL    :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(BUF_BYTES - MAX_XFER_BYTES    , BUF_DEPTH+1));
    constant  I_BUF_READY_LEVEL     :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(BUF_BYTES - 4*AXI_DATA_WIDTH/8, BUF_DEPTH+1));
    constant  I_MAX_REQ_SIZE        :  integer := IMAGE_SHAPE.X.MAX_SIZE * IMAGE_SHAPE.C.MAX_SIZE * IMAGE_SHAPE.ELEM_BITS / 8;
    constant  REQ_SIZE_WIDTH        :  integer := CALC_BITS(I_MAX_REQ_SIZE+1);
    -------------------------------------------------------------------------------
    -- AXI I/F 定数
    -------------------------------------------------------------------------------
    constant  AXI_REQ_PROT          :  AXI4_APROT_TYPE
                                    := std_logic_vector(to_unsigned(AXI_PROT  , AXI4_APROT_WIDTH  ));
    constant  AXI_REQ_QOS           :  AXI4_AQOS_TYPE
                                    := std_logic_vector(to_unsigned(AXI_QOS   , AXI4_AQOS_WIDTH   ));
    constant  AXI_REQ_REGION        :  AXI4_AREGION_TYPE
                                    := std_logic_vector(to_unsigned(AXI_REGION, AXI4_AREGION_WIDTH));
    constant  AXI_REQ_CACHE         :  AXI4_ACACHE_TYPE
                                    := std_logic_vector(to_unsigned(AXI_CACHE , AXI4_ACACHE_WIDTH ));
    constant  AXI_REQ_ID            :  std_logic_vector(AXI_ID_WIDTH -1 downto 0)
                                    := std_logic_vector(to_unsigned(AXI_ID    , AXI_ID_WIDTH      ));
    constant  AXI_REQ_LOCK          :  AXI4_ALOCK_TYPE  := (others => '0');
    constant  AXI_REQ_SPECULATIVE   :  std_logic := '1';
    constant  AXI_REQ_SAFETY        :  std_logic := '0';
    constant  AXI_ALIGNMENT_BITS    :  integer := 32;
    constant  AXI_ACK_REGS          :  integer := 1;
    constant  AXI_RDATA_REGS        :  integer := 3;
    constant  OPEN_INFO_BITS        :  integer := 4;
    constant  CLOSE_INFO_BITS       :  integer := 4;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_tran_start          :  std_logic;
    signal    i_tran_first          :  std_logic;
    signal    i_tran_last           :  std_logic;
    signal    i_tran_addr           :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_tran_addr_load      :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_tran_size           :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_tran_size_load      :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_tran_busy           :  std_logic;
    signal    i_tran_done           :  std_logic;
    signal    i_tran_error          :  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_req_valid           :  std_logic;
    signal    i_req_addr            :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_req_size            :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_req_buf_ptr         :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    i_req_first           :  std_logic;
    signal    i_req_last            :  std_logic;
    signal    i_req_ready           :  std_logic;
    signal    i_ack_valid           :  std_logic;
    signal    i_ack_size            :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_ack_error           :  std_logic;
    signal    i_ack_next            :  std_logic;
    signal    i_ack_last            :  std_logic;
    signal    i_ack_stop            :  std_logic;
    signal    i_ack_none            :  std_logic;
    signal    i_xfer_busy           :  std_logic;
    signal    i_xfer_done           :  std_logic;
    signal    i_xfer_error          :  std_logic;
    signal    i_flow_ready          :  std_logic;
    signal    i_flow_pause          :  std_logic;
    signal    i_flow_stop           :  std_logic;
    signal    i_flow_last           :  std_logic;
    signal    i_flow_size           :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_fin_valid      :  std_logic;
    signal    i_push_fin_last       :  std_logic;
    signal    i_push_fin_error      :  std_logic;
    signal    i_push_fin_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_rsv_valid      :  std_logic;
    signal    i_push_rsv_last       :  std_logic;
    signal    i_push_rsv_error      :  std_logic;
    signal    i_push_rsv_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_buf_reset      :  std_logic;
    signal    i_push_buf_valid      :  std_logic;
    signal    i_push_buf_last       :  std_logic;
    signal    i_push_buf_error      :  std_logic;
    signal    i_push_buf_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_buf_ready      :  std_logic;
    signal    i_open                :  std_logic;
    constant  i_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0) := (others => '0');
    constant  i_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0) := (others => '0');
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    buf_ren               :  std_logic;
    signal    buf_rptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_rdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_wen               :  std_logic;
    signal    buf_wptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_wdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_we                :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    signal    buf_ben               :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    o_open                :  std_logic;
    signal    o_done                :  std_logic;
    signal    o_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0);
    signal    o_open_valid          :  std_logic;
    signal    o_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0);
    signal    o_close_valid         :  std_logic;
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    req_elem_bytes   <= 9 * ((QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD) / 8) when (REQ_K3x3 = '1') else
                        1 * ((QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD) / 8);
    req_image_c_size <= to_integer(to_01(unsigned(REQ_IN_C      )));
    req_slice_c_size <= to_integer(to_01(unsigned(REQ_IN_C      )));
    req_image_x_size <= to_integer(to_01(unsigned(REQ_OUT_C     )));
    req_slice_x_pos  <= to_integer(to_01(unsigned(REQ_OUT_C_POS )));
    req_slice_x_size <= to_integer(to_01(unsigned(REQ_OUT_C_SIZE)));
    req_axi_addr     <= std_logic_vector(resize(unsigned(REQ_ADDR), AXI_ADDR_WIDTH));
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    MST_CTRL: IMAGE_SLICE_MASTER_CONTROLLER              -- 
        generic map (                                    -- 
            SOURCE_SHAPE        => IMAGE_SHAPE         , --
            SLICE_SHAPE         => IMAGE_SHAPE         , --
            MAX_SLICE_C_POS     => 0                   , --
            MAX_SLICE_X_POS     => IMAGE_SHAPE.X.MAX_SIZE , --
            MAX_SLICE_Y_POS     => 0                   , --
            ADDR_BITS           => AXI_ADDR_WIDTH      , --
            SIZE_BITS           => REQ_SIZE_WIDTH        --
        )                                                -- 
        port map (                                       -- 
        -------------------------------------------------------------------------------
        -- クロック&リセット信号
        -------------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            SOURCE_C_SIZE       => req_image_c_size    , -- In  :
            SOURCE_X_SIZE       => req_image_x_size    , -- In  :
            SLICE_C_SIZE        => req_slice_c_size    , -- In  :
            SLICE_X_POS         => req_slice_x_pos     , -- In  :
            SLICE_X_SIZE        => req_slice_x_size    , -- In  :
            ELEM_BYTES          => req_elem_bytes      , -- In  :
            REQ_ADDR            => req_axi_addr        , -- In  :
            REQ_VALID           => REQ_VALID           , -- In  :
            REQ_READY           => REQ_READY           , -- Out :
            RES_NONE            => RES_NONE            , -- Out :
            RES_ERROR           => RES_ERROR           , -- Out :
            RES_VALID           => RES_VALID           , -- Out :
            RES_READY           => RES_READY           , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            MST_ADDR            => i_tran_addr         , -- Out :
            MST_SIZE            => i_tran_size         , -- Out :
            MST_FIRST           => i_tran_first        , -- Out :
            MST_LAST            => i_tran_last         , -- Out :
            MST_START           => i_tran_start        , -- Out :
            MST_BUSY            => i_tran_busy         , -- In  :
            MST_DONE            => i_tran_done         , -- In  :
            MST_ERROR           => i_tran_error          -- In  :
        );                                               -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    i_tran_addr_load <= (others => '1') when (i_tran_start = '1') else (others => '0');
    i_tran_size_load <= (others => '1') when (i_tran_start = '1') else (others => '0');
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    PUMP_CTRL: PUMP_STREAM_INTAKE_CONTROLLER             -- 
        generic map (                                    -- 
            I_CLK_RATE          => 1                   , --
            I_REQ_ADDR_VALID    => I_REQ_ADDR_VALID    , --
            I_REQ_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            I_REG_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            I_REQ_SIZE_VALID    => I_REQ_SIZE_VALID    , --
            I_REQ_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            I_REG_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            I_REG_MODE_BITS     => 1                   , --
            I_REG_STAT_BITS     => 1                   , --
            I_USE_PUSH_BUF_SIZE => I_USE_PUSH_BUF_SIZE , --
            I_FIXED_FLOW_OPEN   => I_FIXED_FLOW_OPEN   , --
            I_FIXED_POOL_OPEN   => I_FIXED_POOL_OPEN   , --
            O_CLK_RATE          => 1                   , --
            O_DATA_BITS         => O_DATA'length       , --
            BUF_DEPTH           => BUF_DEPTH           , --
            BUF_DATA_BITS       => BUF_WIDTH           , --
            I2O_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            I2O_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            O2I_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            O2I_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            I2O_DELAY_CYCLE     => 1                     --
        )                                                -- 
        port map (                                       -- 
        ---------------------------------------------------------------------------
        --Reset Signals.
        ---------------------------------------------------------------------------
            RST                 => RST                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Clock and Clock Enable.
        ---------------------------------------------------------------------------
            I_CLK               => CLK                 , --  In  :
            I_CLR               => CLR                 , --  In  :
            I_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Control Register Interface.
        ---------------------------------------------------------------------------
            I_ADDR_L            => i_tran_addr_load    , --  In  :
            I_ADDR_D            => i_tran_addr         , --  In  :
            I_SIZE_L            => i_tran_size_load    , --  In  :
            I_SIZE_D            => i_tran_size         , --  In  :
            I_START_L           => i_tran_start        , --  In  :
            I_START_D           => i_tran_start        , --  In  :
            I_FIRST_L           => i_tran_start        , --  In  :
            I_FIRST_D           => i_tran_first        , --  In  :
            I_LAST_L            => i_tran_start        , --  In  :
            I_LAST_D            => i_tran_last         , --  In  :
            I_DONE_EN_L         => i_tran_start        , --  In  :
            I_DONE_EN_D         => '0'                 , --  In  :
            I_DONE_ST_L         => i_tran_start        , --  In  :
            I_DONE_ST_D         => '0'                 , --  In  :
            I_ERR_ST_L          => i_tran_start        , --  In  :
            I_ERR_ST_D          => '0'                 , --  In  :
            I_CLOSE_ST_L        => i_tran_start        , --  In  :
            I_CLOSE_ST_D        => '0'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Configuration Signals.
        ---------------------------------------------------------------------------
            I_BUF_READY_LEVEL   => I_BUF_READY_LEVEL   , --  In  :
            I_FLOW_READY_LEVEL  => I_FLOW_READY_LEVEL  , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transaction Command Request Signals.
        ---------------------------------------------------------------------------
            I_REQ_VALID         => i_req_valid         , --  Out :
            I_REQ_ADDR          => i_req_addr          , --  Out :
            I_REQ_SIZE          => i_req_size          , --  Out :
            I_REQ_BUF_PTR       => i_req_buf_ptr       , --  Out :
            I_REQ_FIRST         => i_req_first         , --  Out :
            I_REQ_LAST          => i_req_last          , --  Out :
            I_REQ_READY         => i_req_ready         , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transaction Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            I_ACK_VALID         => i_ack_valid         , --  In  :
            I_ACK_SIZE          => i_ack_size          , --  In  :
            I_ACK_ERROR         => i_ack_error         , --  In  :
            I_ACK_NEXT          => i_ack_next          , --  In  :
            I_ACK_LAST          => i_ack_last          , --  In  :
            I_ACK_STOP          => i_ack_stop          , --  In  :
            I_ACK_NONE          => i_ack_none          , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transfer Status Signals.
        ---------------------------------------------------------------------------
            I_XFER_BUSY         => i_xfer_busy         , --  In  :
            I_XFER_DONE         => i_xfer_done         , --  In  :
            I_XFER_ERROR        => i_xfer_error        , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Flow Control Signals.
        ---------------------------------------------------------------------------
            I_FLOW_READY        => i_flow_ready        , --  Out :
            I_FLOW_PAUSE        => i_flow_pause        , --  Out :
            I_FLOW_STOP         => i_flow_stop         , --  Out :
            I_FLOW_LAST         => i_flow_last         , --  Out :
            I_FLOW_SIZE         => i_flow_size         , --  Out :
            I_PUSH_FIN_VALID    => i_push_fin_valid    , --  In  :
            I_PUSH_FIN_LAST     => i_push_fin_last     , --  In  :
            I_PUSH_FIN_ERROR    => i_push_fin_error    , --  In  :
            I_PUSH_FIN_SIZE     => i_push_fin_size     , --  In  :
            I_PUSH_RSV_VALID    => i_push_rsv_valid    , --  In  :
            I_PUSH_RSV_LAST     => i_push_rsv_last     , --  In  :
            I_PUSH_RSV_ERROR    => i_push_rsv_error    , --  In  :
            I_PUSH_RSV_SIZE     => i_push_rsv_size     , --  In  :
            I_PUSH_BUF_RESET    => i_push_buf_reset    , --  In  :
            I_PUSH_BUF_VALID    => i_push_buf_valid    , --  In  :
            I_PUSH_BUF_LAST     => i_push_buf_last     , --  In  :
            I_PUSH_BUF_ERROR    => i_push_buf_error    , --  In  :
            I_PUSH_BUF_SIZE     => i_push_buf_size     , --  In  :
            I_PUSH_BUF_READY    => i_push_buf_ready    , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Status.
        ---------------------------------------------------------------------------
            I_OPEN              => i_open              , --  Out :
            I_TRAN_BUSY         => i_tran_busy         , --  Out :
            I_TRAN_DONE         => i_tran_done         , --  Out :
            I_TRAN_ERROR        => i_tran_error        , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            I_I2O_OPEN_INFO     => i_open_info         , --  In  :
            I_I2O_CLOSE_INFO    => i_close_info        , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Clock and Clock Enable.
        ---------------------------------------------------------------------------
            O_CLK               => CLK                 , --  In  :
            O_CLR               => CLR                 , --  In  :
            O_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Stream Interface.
        ---------------------------------------------------------------------------
            O_DATA              => O_DATA              , --  Out :
            O_STRB              => open                , --  Out :
            O_LAST              => O_LAST              , --  Out :
            O_VALID             => O_VALID             , --  Out :
            O_READY             => O_READY             , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            O_O2I_OPEN_INFO     => o_open_info         , --  In  :
            O_O2I_OPEN_VALID    => o_open_valid        , --  In  :
            O_O2I_CLOSE_INFO    => o_close_info        , --  In  :
            O_O2I_CLOSE_VALID   => o_close_valid       , --  In  :
            O_I2O_OPEN_INFO     => o_open_info         , --  Out :
            O_I2O_OPEN_VALID    => o_open_valid        , --  Out :
            O_I2O_CLOSE_INFO    => o_close_info        , --  Out :
            O_I2O_CLOSE_VALID   => o_close_valid       , --  Out :
        ---------------------------------------------------------------------------
        -- Outlet Buffer Read Interface.
        ---------------------------------------------------------------------------
            BUF_REN             => buf_ren             , --  Out :
            BUF_PTR             => buf_rptr            , --  Out :
            BUF_DATA            => buf_rdata             --  In  :
        );                                               --
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    AXI_IF: AXI4_MASTER_READ_INTERFACE                   -- 
        generic map (                                    -- 
            AXI4_ADDR_WIDTH     => AXI_ADDR_WIDTH      , -- 
            AXI4_DATA_WIDTH     => AXI_DATA_WIDTH      , --   
            AXI4_ID_WIDTH       => AXI_ID_WIDTH        , --   
            VAL_BITS            => 1                   , --   
            REQ_SIZE_BITS       => REQ_SIZE_WIDTH      , --   
            REQ_SIZE_VALID      => 1                   , --   
            FLOW_VALID          => I_FLOW_VALID        , --   
            BUF_DATA_WIDTH      => BUF_WIDTH           , --   
            BUF_PTR_BITS        => BUF_DEPTH           , --   
            ALIGNMENT_BITS      => AXI_ALIGNMENT_BITS  , --   
            XFER_SIZE_BITS      => BUF_DEPTH+1         , --   
            XFER_MIN_SIZE       => MAX_XFER_SIZE       , --   
            XFER_MAX_SIZE       => MAX_XFER_SIZE       , --   
            QUEUE_SIZE          => AXI_REQ_QUEUE       , --   
            RDATA_REGS          => AXI_RDATA_REGS      , --   
            ACK_REGS            => AXI_ACK_REGS          --   
        )                                                -- 
        port map(                                        --
        ---------------------------------------------------------------------------
        -- Clock and Reset Signals.
        ---------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            ARID                => AXI_ARID            , -- Out :
            ARADDR              => AXI_ARADDR          , -- Out :
            ARLEN               => AXI_ARLEN           , -- Out :
            ARSIZE              => AXI_ARSIZE          , -- Out :
            ARBURST             => AXI_ARBURST         , -- Out :
            ARLOCK              => AXI_ARLOCK          , -- Out :
            ARCACHE             => AXI_ARCACHE         , -- Out :
            ARPROT              => AXI_ARPROT          , -- Out :
            ARQOS               => AXI_ARQOS           , -- Out :
            ARREGION            => AXI_ARREGION        , -- Out :
            ARVALID             => AXI_ARVALID         , -- Out :
            ARREADY             => AXI_ARREADY         , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            RID                 => AXI_RID             , -- In  :
            RDATA               => AXI_RDATA           , -- In  :
            RRESP               => AXI_RRESP           , -- In  :
            RLAST               => AXI_RLAST           , -- In  :
            RVALID              => AXI_RVALID          , -- In  :
            RREADY              => AXI_RREADY          , -- Out :
        ---------------------------------------------------------------------------
        -- Command Request Signals.
        ---------------------------------------------------------------------------
            XFER_SIZE_SEL       => "1"                 , -- In  :
            REQ_ADDR            => i_req_addr          , -- In  :
            REQ_SIZE            => i_req_size          , -- In  :
            REQ_ID              => AXI_REQ_ID          , -- In  :
            REQ_BURST           => AXI4_ABURST_INCR    , -- In  :
            REQ_LOCK            => AXI_REQ_LOCK        , -- In  :
            REQ_CACHE           => AXI_REQ_CACHE       , -- In  :
            REQ_PROT            => AXI_REQ_PROT        , -- In  :
            REQ_QOS             => AXI_REQ_QOS         , -- In  :
            REQ_REGION          => AXI_REQ_REGION      , -- In  :
            REQ_BUF_PTR         => i_req_buf_ptr       , -- In  :
            REQ_FIRST           => i_req_first         , -- In  :
            REQ_LAST            => i_req_last          , -- In  :
            REQ_SPECULATIVE     => AXI_REQ_SPECULATIVE , -- In  :
            REQ_SAFETY          => AXI_REQ_SAFETY      , -- In  :
            REQ_VAL(0)          => i_req_valid         , -- In  :
            REQ_RDY             => i_req_ready         , -- Out :
        ---------------------------------------------------------------------------
        -- Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            ACK_VAL(0)          => i_ack_valid         , -- Out :
            ACK_NEXT            => i_ack_next          , -- Out :
            ACK_LAST            => i_ack_last          , -- Out :
            ACK_ERROR           => i_ack_error         , -- Out :
            ACK_STOP            => i_ack_stop          , -- Out :
            ACK_NONE            => i_ack_none          , -- Out :
            ACK_SIZE            => i_ack_size          , -- Out :
        ---------------------------------------------------------------------------
        -- Transfer Status Signal.
        ---------------------------------------------------------------------------
            XFER_BUSY(0)        => i_xfer_busy         , -- Out :
            XFER_ERROR(0)       => i_xfer_error        , -- Out :
            XFER_DONE(0)        => i_xfer_done         , -- Out :
        ---------------------------------------------------------------------------
        -- Flow Control Signals.
        ---------------------------------------------------------------------------
            FLOW_STOP           => i_flow_stop         , -- In  :
            FLOW_PAUSE          => i_flow_pause        , -- In  :
            FLOW_LAST           => i_flow_last         , -- In  :
            FLOW_SIZE           => i_flow_size         , -- In  :
        ---------------------------------------------------------------------------
        -- Push Reserve Size Signals.
        ---------------------------------------------------------------------------
            PUSH_RSV_VAL(0)     => i_push_rsv_valid    , -- Out :
            PUSH_RSV_LAST       => i_push_rsv_last     , -- Out :
            PUSH_RSV_ERROR      => i_push_rsv_error    , -- Out :
            PUSH_RSV_SIZE       => i_push_rsv_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Push Final Size Signals.
        ---------------------------------------------------------------------------
            PUSH_FIN_VAL(0)     => i_push_fin_valid    , -- Out :
            PUSH_FIN_LAST       => i_push_fin_last     , -- Out :
            PUSH_FIN_ERROR      => i_push_fin_error    , -- Out :
            PUSH_FIN_SIZE       => i_push_fin_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Push Buffer Size Signals.
        ---------------------------------------------------------------------------
            PUSH_BUF_RESET(0)   => i_push_buf_reset    , -- Out :
            PUSH_BUF_VAL(0)     => i_push_buf_valid    , -- Out :
            PUSH_BUF_LAST       => i_push_buf_last     , -- Out :
            PUSH_BUF_ERROR      => i_push_buf_error    , -- Out :
            PUSH_BUF_SIZE       => i_push_buf_size     , -- Out :
            PUSH_BUF_RDY(0)     => i_push_buf_ready    , -- In  :
        ---------------------------------------------------------------------------
        -- Read Buffer Interface Signals.
        ---------------------------------------------------------------------------
            BUF_WEN(0)          => buf_wen             , -- Out :
            BUF_BEN             => buf_ben             , -- Out :
            BUF_DATA            => buf_wdata           , -- Out :
            BUF_PTR             => buf_wptr              -- Out :
        );                                               -- 
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    RAM: SDPRAM 
        generic map(
            DEPTH       => BUF_DEPTH+3         ,
            RWIDTH      => BUF_DATA_BIT_SIZE   , --
            WWIDTH      => BUF_DATA_BIT_SIZE   , --
            WEBIT       => BUF_DATA_BIT_SIZE-3 , --
            ID          => 0                     -- 
        )                                        -- 
        port map (                               -- 
            WCLK        => CLK                 , -- In  :
            WE          => buf_we              , -- In  :
            WADDR       => buf_wptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            WDATA       => buf_wdata           , -- In  :
            RCLK        => CLK                 , -- In  :
            RADDR       => buf_rptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            RDATA       => buf_rdata             -- Out :
        );
    buf_we <= buf_ben when (buf_wen = '1') else (others => '0');
end RTL;

-----------------------------------------------------------------------------------
--!     @file    qconv_strip_out_data_axi_writer.vhd
--!     @brief   Quantized Convolution (strip) Out Data AXI Writer Module
--!     @version 0.1.0
--!     @date    2019/4/15
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_OUT_DATA_AXI_WRITER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 128*(64/8);
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        I_DATA_WIDTH    : --! @brief STREAM DATA WIDTH :
                          integer := 32;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_AWID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_AWADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_AWLEN       : out std_logic_vector(7 downto 0);
        AXI_AWSIZE      : out std_logic_vector(2 downto 0);
        AXI_AWBURST     : out std_logic_vector(1 downto 0);
        AXI_AWLOCK      : out std_logic_vector(0 downto 0);
        AXI_AWCACHE     : out std_logic_vector(3 downto 0);
        AXI_AWPROT      : out std_logic_vector(2 downto 0);
        AXI_AWQOS       : out std_logic_vector(3 downto 0);
        AXI_AWREGION    : out std_logic_vector(3 downto 0);
        AXI_AWUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_AWVALID     : out std_logic;
        AXI_AWREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_WID         : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_WDATA       : out std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_WSTRB       : out std_logic_vector(AXI_DATA_WIDTH/8-1 downto 0);
        AXI_WLAST       : out std_logic;
        AXI_WVALID      : out std_logic;
        AXI_WREADY      : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        AXI_BID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_BRESP       : in  std_logic_vector(1 downto 0);
        AXI_BVALID      : in  std_logic;
        AXI_BREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Slave Interface.
    -------------------------------------------------------------------------------
        I_DATA          : in  std_logic_vector(I_DATA_WIDTH    -1 downto 0);
        I_STRB          : in  std_logic_vector(I_DATA_WIDTH/8  -1 downto 0) := (others => '1');
        I_LAST          : in  std_logic;
        I_VALID         : in  std_logic;
        I_READY         : out std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_OUT_C       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_OUT_W       : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        REQ_OUT_H       : in  std_logic_vector(QCONV_PARAM.OUT_H_BITS-1 downto 0);
        REQ_C_POS       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_C_SIZE      : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_X_POS       : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        REQ_X_SIZE      : in  std_logic_vector(QCONV_PARAM.OUT_W_BITS-1 downto 0);
        REQ_USE_TH      : in  std_logic;
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end QCONV_STRIP_OUT_DATA_AXI_WRITER;
-----------------------------------------------------------------------------------
-- アーキテクチャ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.AXI4_TYPES.all;
use     PIPEWORK.AXI4_COMPONENTS.AXI4_MASTER_WRITE_INTERFACE;
use     PIPEWORK.PUMP_COMPONENTS.PUMP_STREAM_OUTLET_CONTROLLER;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_SLICE_MASTER_CONTROLLER;
use     PIPEWORK.COMPONENTS.SDPRAM;
architecture RTL of QCONV_STRIP_OUT_DATA_AXI_WRITER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MAX(A,B: integer) return integer is
    begin
        if (A > B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MAX(A,B,C: integer) return integer is
    begin
        return MAX(A,MAX(B,C));
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B: integer) return integer is
    begin
        if (A < B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B,C: integer) return integer is
    begin
        return MIN(A,MIN(B,C));
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function CALC_BITS(SIZE:integer) return integer is
        variable bits : integer;
    begin
        bits := 0;
        while (2**bits < SIZE) loop
            bits := bits + 1;
        end loop;
        return bits;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  IMAGE_SHAPE           :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                           ELEM_BITS => MAX(QCONV_PARAM.NBITS_OUT_DATA, QCONV_PARAM.NBITS_IN_DATA, I_DATA_WIDTH),
                                           C         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_OUT_C),
                                           X         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_OUT_W),
                                           Y         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_OUT_H)
                                       );
    signal    req_image_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_image_x_size      :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_image_y_size      :  integer range 0 to IMAGE_SHAPE.Y.MAX_SIZE;
    signal    req_slice_c_pos       :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_slice_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_slice_x_pos       :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_slice_x_size      :  integer range 0 to IMAGE_SHAPE.X.MAX_SIZE;
    signal    req_elem_bytes        :  integer range 0 to IMAGE_SHAPE.ELEM_BITS/8;
    signal    req_axi_addr          :  std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    -------------------------------------------------------------------------------
    -- 一回のトランザクションで転送する最大転送バイト数
    -------------------------------------------------------------------------------
    constant  MAX_XFER_BYTES        :  integer := MIN(4096, 256*(AXI_DATA_WIDTH/8), 2**AXI_XFER_SIZE);
    constant  MAX_XFER_SIZE         :  integer := CALC_BITS(MAX_XFER_BYTES);
    ------------------------------------------------------------------------------
    -- バッファの容量をバイト数で示す.
    ------------------------------------------------------------------------------
    constant  BUF_BYTES             :  integer := MAX_XFER_BYTES*2;
    ------------------------------------------------------------------------------
    -- バッファの容量(バイト数)を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DEPTH             :  integer := CALC_BITS(BUF_BYTES);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を示す.
    ------------------------------------------------------------------------------
    constant  BUF_WIDTH             :  integer := MAX(AXI_DATA_WIDTH, I_DATA_WIDTH);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DATA_BIT_SIZE     :  integer := CALC_BITS(BUF_WIDTH);
    ------------------------------------------------------------------------------
    -- 入力側のフロー制御用定数.
    ------------------------------------------------------------------------------
    constant  O_FLOW_VALID          :  integer := 1;
    constant  O_USE_PULL_BUF_SIZE   :  integer := 0;
    constant  O_FIXED_FLOW_OPEN     :  integer := 0;
    constant  O_FIXED_POOL_OPEN     :  integer := 1;
    constant  O_REQ_ADDR_VALID      :  integer := 1;
    constant  O_REQ_SIZE_VALID      :  integer := 1;
    constant  O_BUF_READY_LEVEL     :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(AXI_DATA_WIDTH/8, BUF_DEPTH+1));
    constant  O_MAX_REQ_SIZE        :  integer := IMAGE_SHAPE.X.MAX_SIZE * IMAGE_SHAPE.C.MAX_SIZE * IMAGE_SHAPE.ELEM_BITS / 8;
    constant  REQ_SIZE_WIDTH        :  integer := CALC_BITS(O_MAX_REQ_SIZE+1);
    -------------------------------------------------------------------------------
    -- AXI I/F 定数
    -------------------------------------------------------------------------------
    constant  AXI_REQ_PROT          :  AXI4_APROT_TYPE
                                    := std_logic_vector(to_unsigned(AXI_PROT  , AXI4_APROT_WIDTH  ));
    constant  AXI_REQ_QOS           :  AXI4_AQOS_TYPE
                                    := std_logic_vector(to_unsigned(AXI_QOS   , AXI4_AQOS_WIDTH   ));
    constant  AXI_REQ_REGION        :  AXI4_AREGION_TYPE
                                    := std_logic_vector(to_unsigned(AXI_REGION, AXI4_AREGION_WIDTH));
    constant  AXI_REQ_CACHE         :  AXI4_ACACHE_TYPE
                                    := std_logic_vector(to_unsigned(AXI_CACHE , AXI4_ACACHE_WIDTH ));
    constant  AXI_REQ_ID            :  std_logic_vector(AXI_ID_WIDTH -1 downto 0)
                                    := std_logic_vector(to_unsigned(AXI_ID    , AXI_ID_WIDTH      ));
    constant  AXI_REQ_LOCK          :  AXI4_ALOCK_TYPE  := (others => '0');
    constant  AXI_REQ_SPECULATIVE   :  std_logic := '1';
    constant  AXI_REQ_SAFETY        :  std_logic := '0';
    constant  AXI_ALIGNMENT_BITS    :  integer := 32;
    constant  AXI_REQ_REGS          :  integer := 1;
    constant  AXI_ACK_REGS          :  integer := 1;
    constant  AXI_RESP_REGS         :  integer := 1;
    constant  OPEN_INFO_BITS        :  integer := 4;
    constant  CLOSE_INFO_BITS       :  integer := 4;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    o_tran_start          :  std_logic;
    signal    o_tran_first          :  std_logic;
    signal    o_tran_last           :  std_logic;
    signal    o_tran_addr           :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    o_tran_addr_load      :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    o_tran_size           :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    o_tran_size_load      :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    o_tran_busy           :  std_logic;
    signal    o_tran_done           :  std_logic;
    signal    o_tran_error          :  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    o_req_valid           :  std_logic;
    signal    o_req_addr            :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    o_req_size            :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    o_req_buf_ptr         :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    o_req_first           :  std_logic;
    signal    o_req_last            :  std_logic;
    signal    o_req_ready           :  std_logic;
    signal    o_ack_valid           :  std_logic;
    signal    o_ack_size            :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    o_ack_error           :  std_logic;
    signal    o_ack_next            :  std_logic;
    signal    o_ack_last            :  std_logic;
    signal    o_ack_stop            :  std_logic;
    signal    o_ack_none            :  std_logic;
    signal    o_xfer_busy           :  std_logic;
    signal    o_xfer_done           :  std_logic;
    signal    o_xfer_error          :  std_logic;
    signal    o_flow_ready          :  std_logic;
    signal    o_flow_pause          :  std_logic;
    signal    o_flow_stop           :  std_logic;
    signal    o_flow_last           :  std_logic;
    signal    o_flow_size           :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    o_flow_ready_level    :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    o_pull_fin_valid      :  std_logic;
    signal    o_pull_fin_last       :  std_logic;
    signal    o_pull_fin_error      :  std_logic;
    signal    o_pull_fin_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    o_pull_rsv_valid      :  std_logic;
    signal    o_pull_rsv_last       :  std_logic;
    signal    o_pull_rsv_error      :  std_logic;
    signal    o_pull_rsv_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    o_pull_buf_reset      :  std_logic;
    signal    o_pull_buf_valid      :  std_logic;
    signal    o_pull_buf_last       :  std_logic;
    signal    o_pull_buf_error      :  std_logic;
    signal    o_pull_buf_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    o_pull_buf_ready      :  std_logic;
    signal    o_open                :  std_logic;
    constant  o_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0) := (others => '0');
    constant  o_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0) := (others => '0');
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    buf_ren               :  std_logic;
    signal    buf_rptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_rdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_wen               :  std_logic;
    signal    buf_wptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_wdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_we                :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    signal    buf_ben               :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_open                :  std_logic;
    signal    i_done                :  std_logic;
    signal    i_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0);
    signal    i_open_valid          :  std_logic;
    signal    i_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0);
    signal    i_close_valid         :  std_logic;
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    process (REQ_USE_TH, REQ_OUT_C, REQ_C_POS, REQ_C_SIZE)
        variable elem_bytes   :  integer;
        variable image_c_size :  integer;
        variable slice_c_pos  :  integer;
        variable slice_c_size :  integer;
        procedure CALC_C_SIZE(constant ELEM_BITS: integer) is
        begin
            if    (ELEM_BITS mod I_DATA_WIDTH = 0) then
                elem_bytes   := ELEM_BITS/8;
                image_c_size := to_integer(to_01(unsigned(REQ_OUT_C )));
                slice_c_pos  := to_integer(to_01(unsigned(REQ_C_POS )));
                slice_c_size := to_integer(to_01(unsigned(REQ_C_SIZE)));
            elsif (ELEM_BITS < I_DATA_WIDTH and I_DATA_WIDTH mod ELEM_BITS = 0) then
                elem_bytes   := I_DATA_WIDTH/8;
                image_c_size := to_integer(to_01(unsigned(REQ_OUT_C ))) / (I_DATA_WIDTH / ELEM_BITS);
                slice_c_pos  := to_integer(to_01(unsigned(REQ_C_POS ))) / (I_DATA_WIDTH / ELEM_BITS);
                slice_c_size := to_integer(to_01(unsigned(REQ_C_SIZE))) / (I_DATA_WIDTH / ELEM_BITS);
            else
                assert (FALSE) report string'("Invalid ELEM_BITS") severity FAILURE;
            end if;
        end procedure;
    begin
        if (REQ_USE_TH = '1') then
            CALC_C_SIZE(QCONV_PARAM.NBITS_IN_DATA );
        else
            CALC_C_SIZE(QCONV_PARAM.NBITS_OUT_DATA);
        end if;
        req_elem_bytes   <= elem_bytes;
        req_image_c_size <= image_c_size;
        req_slice_c_pos  <= slice_c_pos;
        req_slice_c_size <= slice_c_size;
    end process;
    req_image_x_size <= to_integer(to_01(unsigned(REQ_OUT_W )));
    req_image_y_size <= to_integer(to_01(unsigned(REQ_OUT_H )));
    req_slice_x_pos  <= to_integer(to_01(unsigned(REQ_X_POS )));
    req_slice_x_size <= to_integer(to_01(unsigned(REQ_X_SIZE)));
    req_axi_addr     <= std_logic_vector(resize(unsigned(REQ_ADDR), AXI_ADDR_WIDTH));
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    MST_CTRL: IMAGE_SLICE_MASTER_CONTROLLER              -- 
        generic map (                                    -- 
            SOURCE_SHAPE        => IMAGE_SHAPE         , --
            SLICE_SHAPE         => IMAGE_SHAPE         , --
            MAX_SLICE_C_POS     => IMAGE_SHAPE.C.MAX_SIZE , --
            MAX_SLICE_X_POS     => IMAGE_SHAPE.X.MAX_SIZE , --
            MAX_SLICE_Y_POS     => 0                   , --
            ADDR_BITS           => AXI_ADDR_WIDTH      , --
            SIZE_BITS           => REQ_SIZE_WIDTH        --
        )                                                -- 
        port map (                                       -- 
        -------------------------------------------------------------------------------
        -- クロック&リセット信号
        -------------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            SOURCE_C_SIZE       => req_image_c_size    , -- In  :
            SOURCE_X_SIZE       => req_image_x_size    , -- In  :
            SOURCE_Y_SIZE       => req_image_y_size    , -- In  :
            SLICE_C_POS         => req_slice_c_pos     , -- In  :
            SLICE_C_SIZE        => req_slice_c_size    , -- In  :
            SLICE_X_POS         => req_slice_x_pos     , -- In  :
            SLICE_X_SIZE        => req_slice_x_size    , -- In  :
            SLICE_Y_SIZE        => req_image_y_size    , -- In  :
            ELEM_BYTES          => req_elem_bytes      , -- In  :
            REQ_ADDR            => req_axi_addr        , -- In  :
            REQ_VALID           => REQ_VALID           , -- In  :
            REQ_READY           => REQ_READY           , -- Out :
            RES_NONE            => RES_NONE            , -- Out :
            RES_ERROR           => RES_ERROR           , -- Out :
            RES_VALID           => RES_VALID           , -- Out :
            RES_READY           => RES_READY           , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            MST_ADDR            => o_tran_addr         , -- Out :
            MST_SIZE            => o_tran_size         , -- Out :
            MST_FIRST           => o_tran_first        , -- Out :
            MST_LAST            => o_tran_last         , -- Out :
            MST_START           => o_tran_start        , -- Out :
            MST_BUSY            => o_tran_busy         , -- In  :
            MST_DONE            => o_tran_done         , -- In  :
            MST_ERROR           => o_tran_error          -- In  :
        );                                               -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    o_tran_addr_load <= (others => '1') when (o_tran_start = '1') else (others => '0');
    o_tran_size_load <= (others => '1') when (o_tran_start = '1') else (others => '0');
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                o_flow_ready_level <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                o_flow_ready_level <= (others => '0');
            elsif (o_tran_start = '1') then
                if (unsigned(o_tran_size) > 2**AXI_XFER_SIZE) then
                    o_flow_ready_level <= std_logic_vector(to_unsigned(2**AXI_XFER_SIZE, o_flow_ready_level'length));
                else
                    o_flow_ready_level <= std_logic_vector(resize(unsigned(o_tran_size), o_flow_ready_level'length));
                end if;
            end if;
        end if;
    end process;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    PUMP_CTRL: PUMP_STREAM_OUTLET_CONTROLLER             -- 
        generic map (                                    -- 
            O_CLK_RATE          => 1                   , --
            O_REQ_ADDR_VALID    => O_REQ_ADDR_VALID    , --
            O_REQ_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            O_REG_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            O_REQ_SIZE_VALID    => O_REQ_SIZE_VALID    , --
            O_REQ_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            O_REG_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            O_REG_MODE_BITS     => 1                   , --
            O_REG_STAT_BITS     => 1                   , --
            O_USE_PULL_BUF_SIZE => O_USE_PULL_BUF_SIZE , --
            O_FIXED_FLOW_OPEN   => O_FIXED_FLOW_OPEN   , --
            O_FIXED_POOL_OPEN   => O_FIXED_POOL_OPEN   , --
            I_CLK_RATE          => 1                   , --
            I_DATA_BITS         => I_DATA_WIDTH        , --
            BUF_DEPTH           => BUF_DEPTH           , --
            BUF_DATA_BITS       => BUF_WIDTH           , --
            O2I_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            O2I_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            I2O_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            I2O_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            I2O_DELAY_CYCLE     => 1                     --
        )                                                -- 
        port map (                                       -- 
        ---------------------------------------------------------------------------
        --Reset Signals.
        ---------------------------------------------------------------------------
            RST                 => RST                 , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Clock and Clock Enable.
        ---------------------------------------------------------------------------
            O_CLK               => CLK                 , --  In  :
            O_CLR               => CLR                 , --  In  :
            O_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Control Register Interface.
        ---------------------------------------------------------------------------
            O_ADDR_L            => o_tran_addr_load    , --  In  :
            O_ADDR_D            => o_tran_addr         , --  In  :
            O_SIZE_L            => o_tran_size_load    , --  In  :
            O_SIZE_D            => o_tran_size         , --  In  :
            O_START_L           => o_tran_start        , --  In  :
            O_START_D           => o_tran_start        , --  In  :
            O_FIRST_L           => o_tran_start        , --  In  :
            O_FIRST_D           => o_tran_first        , --  In  :
            O_LAST_L            => o_tran_start        , --  In  :
            O_LAST_D            => o_tran_last         , --  In  :
            O_DONE_EN_L         => o_tran_start        , --  In  :
            O_DONE_EN_D         => '0'                 , --  In  :
            O_DONE_ST_L         => o_tran_start        , --  In  :
            O_DONE_ST_D         => '0'                 , --  In  :
            O_ERR_ST_L          => o_tran_start        , --  In  :
            O_ERR_ST_D          => '0'                 , --  In  :
            O_CLOSE_ST_L        => o_tran_start        , --  In  :
            O_CLOSE_ST_D        => '0'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Configuration Signals.
        ---------------------------------------------------------------------------
            O_BUF_READY_LEVEL   => O_BUF_READY_LEVEL   , --  In  :
            O_FLOW_READY_LEVEL  => o_flow_ready_level  , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Transaction Command Request Signals.
        ---------------------------------------------------------------------------
            O_REQ_VALID         => o_req_valid         , --  Out :
            O_REQ_ADDR          => o_req_addr          , --  Out :
            O_REQ_SIZE          => o_req_size          , --  Out :
            O_REQ_BUF_PTR       => o_req_buf_ptr       , --  Out :
            O_REQ_FIRST         => o_req_first         , --  Out :
            O_REQ_LAST          => o_req_last          , --  Out :
            O_REQ_READY         => o_req_ready         , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Transaction Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            O_ACK_VALID         => o_ack_valid         , --  In  :
            O_ACK_SIZE          => o_ack_size          , --  In  :
            O_ACK_ERROR         => o_ack_error         , --  In  :
            O_ACK_NEXT          => o_ack_next          , --  In  :
            O_ACK_LAST          => o_ack_last          , --  In  :
            O_ACK_STOP          => o_ack_stop          , --  In  :
            O_ACK_NONE          => o_ack_none          , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Transfer Status Signals.
        ---------------------------------------------------------------------------
            O_XFER_BUSY         => o_xfer_busy         , --  In  :
            O_XFER_DONE         => o_xfer_done         , --  In  :
            O_XFER_ERROR        => o_xfer_error        , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Flow Control Signals.
        ---------------------------------------------------------------------------
            O_FLOW_READY        => o_flow_ready        , --  Out :
            O_FLOW_PAUSE        => o_flow_pause        , --  Out :
            O_FLOW_STOP         => o_flow_stop         , --  Out :
            O_FLOW_LAST         => o_flow_last         , --  Out :
            O_FLOW_SIZE         => o_flow_size         , --  Out :
            O_PULL_FIN_VALID    => o_pull_fin_valid    , --  In  :
            O_PULL_FIN_LAST     => o_pull_fin_last     , --  In  :
            O_PULL_FIN_ERROR    => o_pull_fin_error    , --  In  :
            O_PULL_FIN_SIZE     => o_pull_fin_size     , --  In  :
            O_PULL_RSV_VALID    => o_pull_rsv_valid    , --  In  :
            O_PULL_RSV_LAST     => o_pull_rsv_last     , --  In  :
            O_PULL_RSV_ERROR    => o_pull_rsv_error    , --  In  :
            O_PULL_RSV_SIZE     => o_pull_rsv_size     , --  In  :
            O_PULL_BUF_RESET    => o_pull_buf_reset    , --  In  :
            O_PULL_BUF_VALID    => o_pull_buf_valid    , --  In  :
            O_PULL_BUF_LAST     => o_pull_buf_last     , --  In  :
            O_PULL_BUF_ERROR    => o_pull_buf_error    , --  In  :
            O_PULL_BUF_SIZE     => o_pull_buf_size     , --  In  :
            O_PULL_BUF_READY    => o_pull_buf_ready    , --  Out :
        ---------------------------------------------------------------------------
        -- Outlet Status.
        ---------------------------------------------------------------------------
            O_OPEN              => o_open              , --  Out :
            O_TRAN_BUSY         => o_tran_busy         , --  Out :
            O_TRAN_DONE         => o_tran_done         , --  Out :
            O_TRAN_ERROR        => o_tran_error        , --  Out :
        ---------------------------------------------------------------------------
        -- Outlet Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            O_O2I_OPEN_INFO     => o_open_info         , --  In  :
            O_O2I_CLOSE_INFO    => o_close_info        , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Clock and Clock Enable.
        ---------------------------------------------------------------------------
            I_CLK               => CLK                 , --  In  :
            I_CLR               => CLR                 , --  In  :
            I_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Stream Interface.
        ---------------------------------------------------------------------------
            I_DATA              => I_DATA              , --  In  :
            I_STRB              => I_STRB              , --  In  :
            I_LAST              => I_LAST              , --  In  :
            I_VALID             => I_VALID             , --  In  :
            I_READY             => I_READY             , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Status.
        ---------------------------------------------------------------------------
            I_OPEN              => i_open              , --  Out :
            I_DONE              => i_done              , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            I_I2O_OPEN_INFO     => i_open_info         , --  In  :
            I_I2O_OPEN_VALID    => i_open_valid        , --  In  :
            I_I2O_CLOSE_INFO    => i_close_info        , --  In  :
            I_I2O_CLOSE_VALID   => i_close_valid       , --  In  :
            I_O2I_OPEN_INFO     => i_open_info         , --  Out :
            I_O2I_OPEN_VALID    => i_open_valid        , --  Out :
            I_O2I_CLOSE_INFO    => i_close_info        , --  Out :
            I_O2I_CLOSE_VALID   => i_close_valid       , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Buffer Read Interface.
        ---------------------------------------------------------------------------
            BUF_WEN             => buf_wen             , --  Out :
            BUF_BEN             => buf_ben             , --  Out :
            BUF_PTR             => buf_wptr            , --  Out :
            BUF_DATA            => buf_wdata             --  Out :
        );                                               --
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    AXI_IF: AXI4_MASTER_WRITE_INTERFACE                  -- 
        generic map (                                    -- 
            AXI4_ADDR_WIDTH     => AXI_ADDR_WIDTH      , -- 
            AXI4_DATA_WIDTH     => AXI_DATA_WIDTH      , --   
            AXI4_ID_WIDTH       => AXI_ID_WIDTH        , --   
            VAL_BITS            => 1                   , --   
            REQ_SIZE_BITS       => REQ_SIZE_WIDTH      , --   
            REQ_SIZE_VALID      => 1                   , --   
            FLOW_VALID          => O_FLOW_VALID        , --   
            BUF_DATA_WIDTH      => BUF_WIDTH           , --   
            BUF_PTR_BITS        => BUF_DEPTH           , --   
            ALIGNMENT_BITS      => AXI_ALIGNMENT_BITS  , --   
            XFER_SIZE_BITS      => BUF_DEPTH+1         , --   
            XFER_MIN_SIZE       => MAX_XFER_SIZE       , --   
            XFER_MAX_SIZE       => MAX_XFER_SIZE       , --   
            QUEUE_SIZE          => AXI_REQ_QUEUE       , --   
            REQ_REGS            => AXI_REQ_REGS        , --   
            ACK_REGS            => AXI_ACK_REGS        , --   
            RESP_REGS           => AXI_RESP_REGS         --   
        )                                                -- 
        port map(                                        --
        ---------------------------------------------------------------------------
        -- Clock and Reset Signals.
        ---------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        --------------------------------------------------------------------------
        -- AXI4 Write Address Channel Signals.
        --------------------------------------------------------------------------
            AWID                => AXI_AWID            , -- Out :
            AWADDR              => AXI_AWADDR          , -- Out :
            AWLEN               => AXI_AWLEN           , -- Out :
            AWSIZE              => AXI_AWSIZE          , -- Out :
            AWBURST             => AXI_AWBURST         , -- Out :
            AWLOCK              => AXI_AWLOCK          , -- Out :
            AWCACHE             => AXI_AWCACHE         , -- Out :
            AWPROT              => AXI_AWPROT          , -- Out :
            AWQOS               => AXI_AWQOS           , -- Out :
            AWREGION            => AXI_AWREGION        , -- Out :
            AWVALID             => AXI_AWVALID         , -- Out :
            AWREADY             => AXI_AWREADY         , -- In  :
        --------------------------------------------------------------------------
        -- AXI4 Write Data Channel Signals.
        --------------------------------------------------------------------------
            WID                 => AXI_WID             , -- Out :
            WDATA               => AXI_WDATA           , -- Out :
            WSTRB               => AXI_WSTRB           , -- Out :
            WLAST               => AXI_WLAST           , -- Out :
            WVALID              => AXI_WVALID          , -- Out :
            WREADY              => AXI_WREADY          , -- In  :
        --------------------------------------------------------------------------
        -- AXI4 Write Response Channel Signals.
        --------------------------------------------------------------------------
            BID                 => AXI_BID             , -- In  :
            BRESP               => AXI_BRESP           , -- In  :
            BVALID              => AXI_BVALID          , -- In  :
            BREADY              => AXI_BREADY          , -- Out :
        ---------------------------------------------------------------------------
        -- Command Request Signals.
        ---------------------------------------------------------------------------
            XFER_SIZE_SEL       => "1"                 , -- In  :
            REQ_ADDR            => o_req_addr          , -- In  :
            REQ_SIZE            => o_req_size          , -- In  :
            REQ_ID              => AXI_REQ_ID          , -- In  :
            REQ_BURST           => AXI4_ABURST_INCR    , -- In  :
            REQ_LOCK            => AXI_REQ_LOCK        , -- In  :
            REQ_CACHE           => AXI_REQ_CACHE       , -- In  :
            REQ_PROT            => AXI_REQ_PROT        , -- In  :
            REQ_QOS             => AXI_REQ_QOS         , -- In  :
            REQ_REGION          => AXI_REQ_REGION      , -- In  :
            REQ_BUF_PTR         => o_req_buf_ptr       , -- In  :
            REQ_FIRST           => o_req_first         , -- In  :
            REQ_LAST            => o_req_last          , -- In  :
            REQ_SPECULATIVE     => AXI_REQ_SPECULATIVE , -- In  :
            REQ_SAFETY          => AXI_REQ_SAFETY      , -- In  :
            REQ_VAL(0)          => o_req_valid         , -- In  :
            REQ_RDY             => o_req_ready         , -- Out :
        ---------------------------------------------------------------------------
        -- Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            ACK_VAL(0)          => o_ack_valid         , -- Out :
            ACK_NEXT            => o_ack_next          , -- Out :
            ACK_LAST            => o_ack_last          , -- Out :
            ACK_ERROR           => o_ack_error         , -- Out :
            ACK_STOP            => o_ack_stop          , -- Out :
            ACK_NONE            => o_ack_none          , -- Out :
            ACK_SIZE            => o_ack_size          , -- Out :
        ---------------------------------------------------------------------------
        -- Transfer Status Signal.
        ---------------------------------------------------------------------------
            XFER_BUSY(0)        => o_xfer_busy         , -- Out :
            XFER_ERROR(0)       => o_xfer_error        , -- Out :
            XFER_DONE(0)        => o_xfer_done         , -- Out :
        ---------------------------------------------------------------------------
        -- Flow Control Signals.
        ---------------------------------------------------------------------------
            FLOW_STOP           => o_flow_stop         , -- In  :
            FLOW_PAUSE          => o_flow_pause        , -- In  :
            FLOW_LAST           => o_flow_last         , -- In  :
            FLOW_SIZE           => o_flow_size         , -- In  :
        ---------------------------------------------------------------------------
        -- Pull Reserve Size Signals.
        ---------------------------------------------------------------------------
            PULL_RSV_VAL(0)     => o_pull_rsv_valid    , -- Out :
            PULL_RSV_LAST       => o_pull_rsv_last     , -- Out :
            PULL_RSV_ERROR      => o_pull_rsv_error    , -- Out :
            PULL_RSV_SIZE       => o_pull_rsv_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Pull Final Size Signals.
        ---------------------------------------------------------------------------
            PULL_FIN_VAL(0)     => o_pull_fin_valid    , -- Out :
            PULL_FIN_LAST       => o_pull_fin_last     , -- Out :
            PULL_FIN_ERROR      => o_pull_fin_error    , -- Out :
            PULL_FIN_SIZE       => o_pull_fin_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Pull Buffer Size Signals.
        ---------------------------------------------------------------------------
            PULL_BUF_RESET(0)   => o_pull_buf_reset    , -- Out :
            PULL_BUF_VAL(0)     => o_pull_buf_valid    , -- Out :
            PULL_BUF_LAST       => o_pull_buf_last     , -- Out :
            PULL_BUF_ERROR      => o_pull_buf_error    , -- Out :
            PULL_BUF_SIZE       => o_pull_buf_size     , -- Out :
            PULL_BUF_RDY(0)     => o_pull_buf_ready    , -- In  :
        ---------------------------------------------------------------------------
        -- Read Buffer Interface Signals.
        ---------------------------------------------------------------------------
            BUF_REN(0)          => buf_ren             , -- Out :
            BUF_DATA            => buf_rdata           , -- Out :
            BUF_PTR             => buf_rptr              -- Out :
        );                                               -- 
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    RAM: SDPRAM 
        generic map(
            DEPTH       => BUF_DEPTH+3         ,
            RWIDTH      => BUF_DATA_BIT_SIZE   , --
            WWIDTH      => BUF_DATA_BIT_SIZE   , --
            WEBIT       => BUF_DATA_BIT_SIZE-3 , --
            ID          => 0                     -- 
        )                                        -- 
        port map (                               -- 
            WCLK        => CLK                 , -- In  :
            WE          => buf_we              , -- In  :
            WADDR       => buf_wptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            WDATA       => buf_wdata           , -- In  :
            RCLK        => CLK                 , -- In  :
            RADDR       => buf_rptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            RDATA       => buf_rdata             -- Out :
        );
    buf_we <= buf_ben when (buf_wen = '1') else (others => '0');
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_registers.vhd
--!     @brief   Quantized Convolution (strip) Registers Module
--!     @version 0.1.0
--!     @date    2019/4/21
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_REGISTERS is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        ID              : --! @brief REGISTER ID STRING :
                          string(1 to 8) := "QCONV-S1";
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        DATA_ADDR_WIDTH : --! @brief I_DATA_ADDR/K_DATA_ADDR/T_DATA_ADDR/O_DATA_ADDR WIDTH :
                          integer := 64;
        REGS_ADDR_WIDTH : --! @brief REGISTER ADDRESS WIDTH :
                          --! レジスタアクセスインターフェースのアドレスのビット数.
                          integer := 7;
        REGS_DATA_WIDTH : --! @brief REGISTER ADDRESS WIDTH :
                          --! レジスタアクセスインターフェースのデータのビット数.
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- クロック&リセット信号
    -------------------------------------------------------------------------------
        CLK             : --! @brief CLOCK :
                          --! クロック信号
                          in  std_logic; 
        RST             : --! @brief ASYNCRONOUSE RESET :
                          --! 非同期リセット信号.アクティブハイ.
                          in  std_logic;
        CLR             : --! @brief SYNCRONOUSE RESET :
                          --! 同期リセット信号.アクティブハイ.
                          in  std_logic;
    -------------------------------------------------------------------------------
    -- Register Access Interface
    -------------------------------------------------------------------------------
        REGS_REQ        : --! @brief REGISTER ACCESS REQUEST :
                          --! レジスタアクセス要求信号.
                          in  std_logic;
        REGS_WRITE      : --! @brief REGISTER WRITE ACCESS :
                          --! レジスタライトアクセス信号.
                          --! * この信号が'1'の時はライトアクセスを行う.
                          --! * この信号が'0'の時はリードアクセスを行う.
                          in  std_logic;
        REGS_ADDR       : --! @brief REGISTER ACCESS ADDRESS :
                          --! レジスタアクセスアドレス信号.
                          in  std_logic_vector(REGS_ADDR_WIDTH  -1 downto 0);
        REGS_BEN        : --! @brief REGISTER BYTE ENABLE :
                          --! レジスタアクセスバイトイネーブル信号.
                          in  std_logic_vector(REGS_DATA_WIDTH/8-1 downto 0);
        REGS_WDATA      : --! @brief REGISTER ACCESS WRITE DATA :
                          --! レジスタアクセスライトデータ.
                          in  std_logic_vector(REGS_DATA_WIDTH  -1 downto 0);
        REGS_RDATA      : --! @brief REGISTER ACCESS READ DATA :
                          --! レジスタアクセスリードデータ.
                          out std_logic_vector(REGS_DATA_WIDTH  -1 downto 0);
        REGS_ACK        : --! @brief REGISTER ACCESS ACKNOWLEDGE :
                          --! レジスタアクセス応答信号.
                          out std_logic;
        REGS_ERR        : --! @brief REGISTER ACCESS ERROR ACKNOWLEDGE :
                          --! レジスタアクセスエラー応答信号.
                          out std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Registers
    -------------------------------------------------------------------------------
        I_DATA_ADDR     : --! @brief IN  DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        O_DATA_ADDR     : --! @brief OUT DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        K_DATA_ADDR     : --! @brief K   DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        T_DATA_ADDR     : --! @brief TH  DATA ADDRESS REGISTER :
                          out std_logic_vector(DATA_ADDR_WIDTH-1 downto 0);
        I_WIDTH         : --! @brief IN  WIDTH REGISTER :
                          out std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
        I_HEIGHT        : --! @brief IN  HEIGHT REGISTER :
                          out std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
        I_CHANNELS      : --! @brief IN  CHANNELS REGISTER :
                          out std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
        O_WIDTH         : --! @brief OUT WIDTH REGISTER :
                          out std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
        O_HEIGHT        : --! @brief OUT HEIGHT REGISTER :
                          out std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
        O_CHANNELS      : --! @brief OUT CHANNELS REGISTER :
                          out std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
        K_WIDTH         : --! @brief K   WIDTH REGISTER :
                          out std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
        K_HEIGHT        : --! @brief K   HEIGHT REGISTER :
                          out std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
        PAD_SIZE        : --! @brief PAD SIZE REGISTER :
                          out std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
        USE_TH          : --! @brief USE THRESHOLD REGISTER :
                          out std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Request/Response Interface
    -------------------------------------------------------------------------------
        REQ_VALID       : --! @brief REQUEST VALID :
                          out std_logic;
        REQ_READY       : --! @brief REQUEST READY :
                          in  std_logic;
        RES_VALID       : --! @brief RESPONSE VALID :
                          in  std_logic;
        RES_READY       : --! @brief RESPONSE READY :
                          out std_logic;
        RES_STATUS      : --! @brief RESPONSE STATUS :
                          in  std_logic;
        REQ_RESET       : --! @brief RESET REQUEST :
                          out std_logic;
        REQ_STOP        : --! @brief STOP REQUEST :
                          out std_logic;
        REQ_PAUSE       : --! @brief PAUSE REQUEST :
                          out std_logic;
    -------------------------------------------------------------------------------
    -- Interrupt Request 
    -------------------------------------------------------------------------------
        IRQ             : --! @brief Interrupt Request :
                          out std_logic
    );
end QCONV_STRIP_REGISTERS;
-----------------------------------------------------------------------------------
-- 
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.COMPONENTS.REGISTER_ACCESS_DECODER;
architecture RTL of QCONV_STRIP_REGISTERS is
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Registers
    -------------------------------------------------------------------------------
    --           31            24              16               8               0
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x00 |                                                       busy--| |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x04 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x08 |                                                      start--| |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x0C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x10 |                                           interrupt enable--| |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x14 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x18 |                             Status[31:00]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x1C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x20 |                   In  Data Address[31:00]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x24 |                   In  Data Address[63:32]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x28 |                   Out Data Address[31:00]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x2C |                   Out Data Address[63:32]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x30 |                   K   Data Address[31:00]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x34 |                   K   Data Address[63:32]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x38 |                   Th  Data Address[31:00]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x3C |                   Th  Data Address[63:32]                     |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x40 |                               |     In  Width  [15:00]        |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x44 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x48 |                               |     In  Height [15:00]        |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x4C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x50 |                               |     In  Channel[15:00]        |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x54 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x58 |                               |     Out Width  [15:00]        |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x5C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x60 |                               |     Out Height [15:00]        |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x64 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x68 |                               |     Out Channel[15:00]        |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x6C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x70 |                                             | K  Width[03:00] |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x74 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x78 |                                             | K Height[03:00] |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x7C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x80 |                                             | Pad Size[03:00] |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x84 |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x88 |                                             |  Use Th [00:00] |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -- Addr=0x8C |                                                               |
    --           +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    -------------------------------------------------------------------------------
    constant  REGS_BASE_ADDR        :  integer := 16#00#;
    constant  REGS_DATA_BITS        :  integer := 16#90# * 8;
    -------------------------------------------------------------------------------
    -- レジスタアクセス用の信号群.
    -------------------------------------------------------------------------------
    signal    regs_load             :  std_logic_vector(REGS_DATA_BITS-1 downto 0);
    signal    regs_wbit             :  std_logic_vector(REGS_DATA_BITS-1 downto 0);
    signal    regs_rbit             :  std_logic_vector(REGS_DATA_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Busy Register
    -------------------------------------------------------------------------------
    -- Busy[63:1]  = 予約.
    -- Busy[0]     = 1: 動作中であることを示す. 0: 待機中であることを示す.
    -------------------------------------------------------------------------------
    constant  BUSY_REGS_ADDR        :  integer := REGS_BASE_ADDR + 16#00#;
    constant  BUSY_BUSY_POS         :  integer := 8*BUSY_REGS_ADDR + 0;
    constant  BUSY_RESV_LO          :  integer := 8*BUSY_REGS_ADDR + 1;
    constant  BUSY_RESV_HI          :  integer := 8*BUSY_REGS_ADDR + 64-1;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Control Register
    -------------------------------------------------------------------------------
    -- Control[63:1] = 予約.
    -- Control[0]    = 1 を書き込むことで動作を開始する.
    -------------------------------------------------------------------------------
    constant  CTRL_REGS_ADDR        :  integer := REGS_BASE_ADDR + 16#08#;
    constant  CTRL_START_POS        :  integer := 8*CTRL_REGS_ADDR + 0;
    constant  CTRL_RESV_LO          :  integer := 8*CTRL_REGS_ADDR + 1;
    constant  CTRL_RESV_HI          :  integer := 8*CTRL_REGS_ADDR + 64-1;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Interrupt Enable Register
    -------------------------------------------------------------------------------
    -- IrqEna[63:1] = 予約.
    -- IrqEna[0]    = 1: 割込みを許可する. 0: 割込みを禁止する.
    -------------------------------------------------------------------------------
    constant  IRQE_REGS_ADDR        :  integer := REGS_BASE_ADDR + 16#10#;
    constant  IRQE_IREQ_POS         :  integer := 8*IRQE_REGS_ADDR + 0;
    constant  IRQE_RESV_LO          :  integer := 8*IRQE_REGS_ADDR + 1;
    constant  IRQE_RESV_HI          :  integer := 8*IRQE_REGS_ADDR + 64-1;
    signal    irq_enable            :  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Status Register
    -------------------------------------------------------------------------------
    -- Status[63:2] = 予約.
    -- Status[1]    = 1: 動作が終了したことを示す.
    -- Status[0]    = 1: 割込みが発生したことを示す.
    -------------------------------------------------------------------------------
    constant  STAT_REGS_ADDR        :  integer := REGS_BASE_ADDR + 16#18#;
    constant  STAT_IRQ_POS          :  integer := 8*STAT_REGS_ADDR + 0;
    constant  STAT_DONE_POS         :  integer := 8*STAT_REGS_ADDR + 1;
    constant  STAT_RESV_LO          :  integer := 8*STAT_REGS_ADDR + 2;
    constant  STAT_RESV_HI          :  integer := 8*STAT_REGS_ADDR + 64-1;
    signal    status_done           :  std_logic;
    signal    status_irq            :  std_logic;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Data Address Register
    -------------------------------------------------------------------------------
    constant  I_DATA_ADDR_REGS_ADDR :  integer := REGS_BASE_ADDR + 16#20#;
    constant  I_DATA_ADDR_REGS_BITS :  integer := 64;
    constant  I_DATA_ADDR_REGS_LO   :  integer := 8*I_DATA_ADDR_REGS_ADDR;
    constant  I_DATA_ADDR_REGS_HI   :  integer := 8*I_DATA_ADDR_REGS_ADDR + I_DATA_ADDR_REGS_BITS-1;
    signal    i_data_addr_regs      :  std_logic_vector(I_DATA_ADDR_REGS_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Data Address Register
    -------------------------------------------------------------------------------
    constant  O_DATA_ADDR_REGS_ADDR :  integer := REGS_BASE_ADDR + 16#28#;
    constant  O_DATA_ADDR_REGS_BITS :  integer := 64;
    constant  O_DATA_ADDR_REGS_LO   :  integer := 8*O_DATA_ADDR_REGS_ADDR;
    constant  O_DATA_ADDR_REGS_HI   :  integer := 8*O_DATA_ADDR_REGS_ADDR + O_DATA_ADDR_REGS_BITS-1;
    signal    o_data_addr_regs      :  std_logic_vector(O_DATA_ADDR_REGS_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) K   Data Address Register
    -------------------------------------------------------------------------------
    constant  K_DATA_ADDR_REGS_ADDR :  integer := REGS_BASE_ADDR + 16#30#;
    constant  K_DATA_ADDR_REGS_BITS :  integer := 64;
    constant  K_DATA_ADDR_REGS_LO   :  integer := 8*K_DATA_ADDR_REGS_ADDR;
    constant  K_DATA_ADDR_REGS_HI   :  integer := 8*K_DATA_ADDR_REGS_ADDR + K_DATA_ADDR_REGS_BITS-1;
    signal    k_data_addr_regs      :  std_logic_vector(K_DATA_ADDR_REGS_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Th  Data Address Register
    -------------------------------------------------------------------------------
    constant  T_DATA_ADDR_REGS_ADDR :  integer := REGS_BASE_ADDR + 16#38#;
    constant  T_DATA_ADDR_REGS_BITS :  integer := 64;
    constant  T_DATA_ADDR_REGS_LO   :  integer := 8*T_DATA_ADDR_REGS_ADDR;
    constant  T_DATA_ADDR_REGS_HI   :  integer := 8*T_DATA_ADDR_REGS_ADDR + T_DATA_ADDR_REGS_BITS-1;
    signal    t_data_addr_regs      :  std_logic_vector(T_DATA_ADDR_REGS_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Width    Register
    -------------------------------------------------------------------------------
    constant  I_WIDTH_REGS_ADDR     :  integer := REGS_BASE_ADDR + 16#40#;
    constant  I_WIDTH_REGS_BITS     :  integer := I_WIDTH'length;
    constant  I_WIDTH_REGS_LO       :  integer := 8*I_WIDTH_REGS_ADDR;
    constant  I_WIDTH_REGS_HI       :  integer := 8*I_WIDTH_REGS_ADDR    + I_WIDTH_REGS_BITS-1;
    constant  I_WIDTH_RESV_LO       :  integer := I_WIDTH_REGS_HI        + 1;
    constant  I_WIDTH_RESV_HI       :  integer := 8*I_WIDTH_REGS_ADDR    + 64-1;
    signal    i_width_regs          :  std_logic_vector(I_WIDTH'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Height   Register
    -------------------------------------------------------------------------------
    constant  I_HEIGHT_REGS_ADDR    :  integer := REGS_BASE_ADDR + 16#48#;
    constant  I_HEIGHT_REGS_BITS    :  integer := I_HEIGHT'length;
    constant  I_HEIGHT_REGS_LO      :  integer := 8*I_HEIGHT_REGS_ADDR;
    constant  I_HEIGHT_REGS_HI      :  integer := 8*I_HEIGHT_REGS_ADDR   + I_HEIGHT_REGS_BITS-1;
    constant  I_HEIGHT_RESV_LO      :  integer := I_HEIGHT_REGS_HI       + 1;
    constant  I_HEIGHT_RESV_HI      :  integer := 8*I_HEIGHT_REGS_ADDR   + 64-1;
    signal    i_height_regs         :  std_logic_vector(I_HEIGHT'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Channels Register
    -------------------------------------------------------------------------------
    constant  I_CHANNELS_REGS_ADDR  :  integer := REGS_BASE_ADDR + 16#50#;
    constant  I_CHANNELS_REGS_BITS  :  integer := I_CHANNELS'length;
    constant  I_CHANNELS_REGS_LO    :  integer := 8*I_CHANNELS_REGS_ADDR;
    constant  I_CHANNELS_REGS_HI    :  integer := 8*I_CHANNELS_REGS_ADDR + I_CHANNELS_REGS_BITS-1;
    constant  I_CHANNELS_RESV_LO    :  integer := I_CHANNELS_REGS_HI     + 1;
    constant  I_CHANNELS_RESV_HI    :  integer := 8*I_CHANNELS_REGS_ADDR + 64-1;
    signal    i_channels_regs       :  std_logic_vector(I_CHANNELS'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Width    Register
    -------------------------------------------------------------------------------
    constant  O_WIDTH_REGS_ADDR     :  integer := REGS_BASE_ADDR + 16#58#;
    constant  O_WIDTH_REGS_BITS     :  integer := O_WIDTH'length;
    constant  O_WIDTH_REGS_LO       :  integer := 8*O_WIDTH_REGS_ADDR;
    constant  O_WIDTH_REGS_HI       :  integer := 8*O_WIDTH_REGS_ADDR    + O_WIDTH_REGS_BITS-1;
    constant  O_WIDTH_RESV_LO       :  integer := O_WIDTH_REGS_HI        + 1;
    constant  O_WIDTH_RESV_HI       :  integer := 8*O_WIDTH_REGS_ADDR    + 64-1;
    signal    o_width_regs          :  std_logic_vector(O_WIDTH'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Height   Register
    -------------------------------------------------------------------------------
    constant  O_HEIGHT_REGS_ADDR    :  integer := REGS_BASE_ADDR + 16#60#;
    constant  O_HEIGHT_REGS_BITS    :  integer := O_HEIGHT'length;
    constant  O_HEIGHT_REGS_LO      :  integer := 8*O_HEIGHT_REGS_ADDR;
    constant  O_HEIGHT_REGS_HI      :  integer := 8*O_HEIGHT_REGS_ADDR   + O_HEIGHT_REGS_BITS-1;
    constant  O_HEIGHT_RESV_LO      :  integer := O_HEIGHT_REGS_HI       + 1;
    constant  O_HEIGHT_RESV_HI      :  integer := 8*O_HEIGHT_REGS_ADDR   + 64-1;
    signal    o_height_regs         :  std_logic_vector(O_HEIGHT'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Channels Register
    -------------------------------------------------------------------------------
    constant  O_CHANNELS_REGS_ADDR  :  integer := REGS_BASE_ADDR + 16#68#;
    constant  O_CHANNELS_REGS_BITS  :  integer := O_CHANNELS'length;
    constant  O_CHANNELS_REGS_LO    :  integer := 8*O_CHANNELS_REGS_ADDR;
    constant  O_CHANNELS_REGS_HI    :  integer := 8*O_CHANNELS_REGS_ADDR + O_CHANNELS_REGS_BITS-1;
    constant  O_CHANNELS_RESV_LO    :  integer := O_CHANNELS_REGS_HI     + 1;
    constant  O_CHANNELS_RESV_HI    :  integer := 8*O_CHANNELS_REGS_ADDR + 64-1;
    signal    o_channels_regs       :  std_logic_vector(O_CHANNELS'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) K   Width    Register
    -------------------------------------------------------------------------------
    constant  K_WIDTH_REGS_ADDR     :  integer := REGS_BASE_ADDR + 16#70#;
    constant  K_WIDTH_REGS_BITS     :  integer := K_WIDTH'length;
    constant  K_WIDTH_REGS_LO       :  integer := 8*K_WIDTH_REGS_ADDR;
    constant  K_WIDTH_REGS_HI       :  integer := 8*K_WIDTH_REGS_ADDR    + K_WIDTH_REGS_BITS-1;
    constant  K_WIDTH_RESV_LO       :  integer := K_WIDTH_REGS_HI        + 1;
    constant  K_WIDTH_RESV_HI       :  integer := 8*K_WIDTH_REGS_ADDR    + 64-1;
    signal    k_width_regs          :  std_logic_vector(K_WIDTH'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) K   Height   Register
    -------------------------------------------------------------------------------
    constant  K_HEIGHT_REGS_ADDR    :  integer := REGS_BASE_ADDR + 16#78#;
    constant  K_HEIGHT_REGS_BITS    :  integer := K_HEIGHT'length;
    constant  K_HEIGHT_REGS_LO      :  integer := 8*K_HEIGHT_REGS_ADDR;
    constant  K_HEIGHT_REGS_HI      :  integer := 8*K_HEIGHT_REGS_ADDR   + K_HEIGHT_REGS_BITS-1;
    constant  K_HEIGHT_RESV_LO      :  integer := K_HEIGHT_REGS_HI       + 1;
    constant  K_HEIGHT_RESV_HI      :  integer := 8*K_HEIGHT_REGS_ADDR   + 64-1;
    signal    k_height_regs         :  std_logic_vector(K_HEIGHT'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Pad Size     Register
    -------------------------------------------------------------------------------
    constant  PAD_SIZE_REGS_ADDR    :  integer := REGS_BASE_ADDR + 16#80#;
    constant  PAD_SIZE_REGS_BITS    :  integer := PAD_SIZE'length;
    constant  PAD_SIZE_REGS_LO      :  integer := 8*PAD_SIZE_REGS_ADDR;
    constant  PAD_SIZE_REGS_HI      :  integer := 8*PAD_SIZE_REGS_ADDR   + PAD_SIZE_REGS_BITS-1;
    constant  PAD_SIZE_RESV_LO      :  integer := PAD_SIZE_REGS_HI       + 1;
    constant  PAD_SIZE_RESV_HI      :  integer := 8*PAD_SIZE_REGS_ADDR   + 64-1;
    signal    pad_size_regs         :  std_logic_vector(PAD_SIZE'range);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Use Threshold Register
    -------------------------------------------------------------------------------
    constant  USE_TH_REGS_ADDR      :  integer := REGS_BASE_ADDR + 16#88#;
    constant  USE_TH_REGS_BITS      :  integer := 1;
    constant  USE_TH_REGS_LO        :  integer := 8*USE_TH_REGS_ADDR;
    constant  USE_TH_REGS_HI        :  integer := 8*USE_TH_REGS_ADDR   + USE_TH_REGS_BITS-1;
    constant  USE_TH_RESV_LO        :  integer := USE_TH_REGS_HI       + 1;
    constant  USE_TH_RESV_HI        :  integer := 8*USE_TH_REGS_ADDR   + 64-1;
    signal    use_th_regs           :  std_logic_vector(USE_TH_REGS_BITS-1 downto 0);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    type      STATE_TYPE            is (IDLE_STATE, REQ_STATE, RUN_STATE, DONE_STATE);
    signal    state                 :  STATE_TYPE;
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    DEC: REGISTER_ACCESS_DECODER             -- 
        generic map (                        -- 
            ADDR_WIDTH  => REGS_ADDR_WIDTH , --
            DATA_WIDTH  => REGS_DATA_WIDTH , --
            WBIT_MIN    => regs_wbit'low   , --
            WBIT_MAX    => regs_wbit'high  , --
            RBIT_MIN    => regs_rbit'low   , --
            RBIT_MAX    => regs_rbit'high    --
        )                                    -- 
        port map (                           -- 
            REGS_REQ    => REGS_REQ        , -- In  :
            REGS_WRITE  => REGS_WRITE      , -- In  :
            REGS_ADDR   => REGS_ADDR       , -- In  :
            REGS_BEN    => REGS_BEN        , -- In  :
            REGS_WDATA  => REGS_WDATA      , -- In  :
            REGS_RDATA  => REGS_RDATA      , -- Out :
            REGS_ACK    => REGS_ACK        , -- Out :
            REGS_ERR    => REGS_ERR        , -- Out :
            W_DATA      => regs_wbit       , -- Out :
            W_LOAD      => regs_load       , -- Out :
            R_DATA      => regs_rbit         -- In  :
        );
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Control/Status Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                state       <= IDLE_STATE;
                irq_enable  <= '0';
                status_done <= '0';
                status_irq  <= '0';
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                state       <= IDLE_STATE;
                irq_enable  <= '0';
                status_done <= '0';
                status_irq  <= '0';
            else
                case state is
                    when IDLE_STATE =>
                        if (regs_load(CTRL_START_POS) = '1' and regs_wbit(CTRL_START_POS) = '1') then
                            state <= REQ_STATE;
                            status_done <= '0';
                        else
                            state <= IDLE_STATE;
                        end if;
                    when REQ_STATE =>
                        if (REQ_READY = '1') then
                            state <= RUN_STATE;
                        else
                            state <= REQ_STATE;
                        end if;
                    when RUN_STATE =>
                        if (RES_VALID = '1') then
                            state <= DONE_STATE;
                        else
                            state <= RUN_STATE;
                        end if;
                    when DONE_STATE =>
                            state <= IDLE_STATE;
                            status_done <= '1';
                    when others => 
                            state <= IDLE_STATE;
                end case;
                if (regs_load(IRQE_IREQ_POS) = '1') then
                    irq_enable <= regs_wbit(IRQE_IREQ_POS);
                end if;
                if    (regs_load(STAT_IRQ_POS) = '1' and regs_wbit(STAT_IRQ_POS) = '1') then
                    status_irq <= '0';
                elsif (state = DONE_STATE) then
                    status_irq <= '1';
                end if;
            end if;
        end if;
    end process;
    REQ_VALID  <= '1' when (state  = REQ_STATE ) else '0';
    RES_READY  <= '1' when (state  = RUN_STATE ) else '0';
    REQ_RESET  <= '0';
    REQ_PAUSE  <= '0';
    REQ_STOP   <= '0';
    regs_rbit(BUSY_BUSY_POS ) <= '1' when (state /= IDLE_STATE) else '0';
    regs_rbit(BUSY_RESV_HI downto BUSY_RESV_LO) <= (BUSY_RESV_HI downto BUSY_RESV_LO => '0');
    regs_rbit(CTRL_START_POS) <= '0';
    regs_rbit(CTRL_RESV_HI downto CTRL_RESV_LO) <= (CTRL_RESV_HI downto CTRL_RESV_LO => '0');
    regs_rbit(IRQE_IREQ_POS ) <= irq_enable;
    regs_rbit(IRQE_RESV_HI downto IRQE_RESV_LO) <= (IRQE_RESV_HI downto IRQE_RESV_LO => '0');
    regs_rbit(STAT_IRQ_POS  ) <= status_irq;
    regs_rbit(STAT_DONE_POS ) <= status_done;
    regs_rbit(STAT_RESV_HI downto STAT_RESV_LO) <= (STAT_RESV_HI downto STAT_RESV_LO => '0');
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Data Address Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                i_data_addr_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                i_data_addr_regs <= (others => '0');
            else
                for pos in i_data_addr_regs'range loop
                    if (regs_load(pos+I_DATA_ADDR_REGS_LO) = '1') then
                        i_data_addr_regs(pos) <= regs_wbit(pos+I_DATA_ADDR_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(I_DATA_ADDR_REGS_HI downto I_DATA_ADDR_REGS_LO) <= i_data_addr_regs;
    I_DATA_ADDR <= i_data_addr_regs(I_DATA_ADDR'high downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Data Address Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                o_data_addr_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                o_data_addr_regs <= (others => '0');
            else
                for pos in o_data_addr_regs'range loop
                    if (regs_load(pos+O_DATA_ADDR_REGS_LO) = '1') then
                        o_data_addr_regs(pos) <= regs_wbit(pos+O_DATA_ADDR_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(O_DATA_ADDR_REGS_HI downto O_DATA_ADDR_REGS_LO) <= o_data_addr_regs;
    O_DATA_ADDR <= o_data_addr_regs(O_DATA_ADDR'high downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) K   Data Address Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                k_data_addr_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                k_data_addr_regs <= (others => '0');
            else
                for pos in k_data_addr_regs'range loop
                    if (regs_load(pos+K_DATA_ADDR_REGS_LO) = '1') then
                        k_data_addr_regs(pos) <= regs_wbit(pos+K_DATA_ADDR_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(K_DATA_ADDR_REGS_HI downto K_DATA_ADDR_REGS_LO) <= k_data_addr_regs;
    K_DATA_ADDR <= k_data_addr_regs(K_DATA_ADDR'high downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Th  Data Address Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                t_data_addr_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                t_data_addr_regs <= (others => '0');
            else
                for pos in t_data_addr_regs'range loop
                    if (regs_load(pos+T_DATA_ADDR_REGS_LO) = '1') then
                        t_data_addr_regs(pos) <= regs_wbit(pos+T_DATA_ADDR_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(T_DATA_ADDR_REGS_HI downto T_DATA_ADDR_REGS_LO) <= t_data_addr_regs;
    T_DATA_ADDR <= t_data_addr_regs(T_DATA_ADDR'high downto 0);
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Width    Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                i_width_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                i_width_regs <= (others => '0');
            else
                for pos in i_width_regs'range loop
                    if (regs_load(pos+I_WIDTH_REGS_LO) = '1') then
                        i_width_regs(pos) <= regs_wbit(pos+I_WIDTH_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(I_WIDTH_REGS_HI downto I_WIDTH_REGS_LO) <= i_width_regs;
    regs_rbit(I_WIDTH_RESV_HI downto I_WIDTH_RESV_LO) <= (I_WIDTH_RESV_HI downto I_WIDTH_RESV_LO => '0');
    I_WIDTH <= i_width_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Height   Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                i_height_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                i_height_regs <= (others => '0');
            else
                for pos in i_height_regs'range loop
                    if (regs_load(pos+I_HEIGHT_REGS_LO) = '1') then
                        i_height_regs(pos) <= regs_wbit(pos+I_HEIGHT_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(I_HEIGHT_REGS_HI downto I_HEIGHT_REGS_LO) <= i_height_regs;
    regs_rbit(I_HEIGHT_RESV_HI downto I_HEIGHT_RESV_LO) <= (I_HEIGHT_RESV_HI downto I_HEIGHT_RESV_LO => '0');
    I_HEIGHT <= i_height_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) In  Channels Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                i_channels_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                i_channels_regs <= (others => '0');
            else
                for pos in i_channels_regs'range loop
                    if (regs_load(pos+I_CHANNELS_REGS_LO) = '1') then
                        i_channels_regs(pos) <= regs_wbit(pos+I_CHANNELS_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(I_CHANNELS_REGS_HI downto I_CHANNELS_REGS_LO) <= i_channels_regs;
    regs_rbit(I_CHANNELS_RESV_HI downto I_CHANNELS_RESV_LO) <= (I_CHANNELS_RESV_HI downto I_CHANNELS_RESV_LO => '0');
    I_CHANNELS <= i_channels_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Width    Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                o_width_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                o_width_regs <= (others => '0');
            else
                for pos in o_width_regs'range loop
                    if (regs_load(pos+O_WIDTH_REGS_LO) = '1') then
                        o_width_regs(pos) <= regs_wbit(pos+O_WIDTH_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(O_WIDTH_REGS_HI downto O_WIDTH_REGS_LO) <= o_width_regs;
    regs_rbit(O_WIDTH_RESV_HI downto O_WIDTH_RESV_LO) <= (O_WIDTH_RESV_HI downto O_WIDTH_RESV_LO => '0');
    O_WIDTH <= o_width_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Height   Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                o_height_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                o_height_regs <= (others => '0');
            else
                for pos in o_height_regs'range loop
                    if (regs_load(pos+O_HEIGHT_REGS_LO) = '1') then
                        o_height_regs(pos) <= regs_wbit(pos+O_HEIGHT_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(O_HEIGHT_REGS_HI downto O_HEIGHT_REGS_LO) <= o_height_regs;
    regs_rbit(O_HEIGHT_RESV_HI downto O_HEIGHT_RESV_LO) <= (O_HEIGHT_RESV_HI downto O_HEIGHT_RESV_LO => '0');
    O_HEIGHT <= o_height_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Out Channels Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                o_channels_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                o_channels_regs <= (others => '0');
            else
                for pos in o_channels_regs'range loop
                    if (regs_load(pos+O_CHANNELS_REGS_LO) = '1') then
                        o_channels_regs(pos) <= regs_wbit(pos+O_CHANNELS_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(O_CHANNELS_REGS_HI downto O_CHANNELS_REGS_LO) <= o_channels_regs;
    regs_rbit(O_CHANNELS_RESV_HI downto O_CHANNELS_RESV_LO) <= (O_CHANNELS_RESV_HI downto O_CHANNELS_RESV_LO => '0');
    O_CHANNELS <= o_channels_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) K   Width    Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                k_width_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                k_width_regs <= (others => '0');
            else
                for pos in k_width_regs'range loop
                    if (regs_load(pos+K_WIDTH_REGS_LO) = '1') then
                        k_width_regs(pos) <= regs_wbit(pos+K_WIDTH_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(K_WIDTH_REGS_HI downto K_WIDTH_REGS_LO) <= k_width_regs;
    regs_rbit(K_WIDTH_RESV_HI downto K_WIDTH_RESV_LO) <= (K_WIDTH_RESV_HI downto K_WIDTH_RESV_LO => '0');
    K_WIDTH <= k_width_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) K   Height   Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                k_height_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                k_height_regs <= (others => '0');
            else
                for pos in k_height_regs'range loop
                    if (regs_load(pos+K_HEIGHT_REGS_LO) = '1') then
                        k_height_regs(pos) <= regs_wbit(pos+K_HEIGHT_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(K_HEIGHT_REGS_HI downto K_HEIGHT_REGS_LO) <= k_height_regs;
    regs_rbit(K_HEIGHT_RESV_HI downto K_HEIGHT_RESV_LO) <= (K_HEIGHT_RESV_HI downto K_HEIGHT_RESV_LO => '0');
    K_HEIGHT <= k_height_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Pad Size     Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                pad_size_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                pad_size_regs <= (others => '0');
            else
                for pos in pad_size_regs'range loop
                    if (regs_load(pos+PAD_SIZE_REGS_LO) = '1') then
                        pad_size_regs(pos) <= regs_wbit(pos+PAD_SIZE_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(PAD_SIZE_REGS_HI downto PAD_SIZE_REGS_LO) <= pad_size_regs;
    regs_rbit(PAD_SIZE_RESV_HI downto PAD_SIZE_RESV_LO) <= (PAD_SIZE_RESV_HI downto PAD_SIZE_RESV_LO => '0');
    PAD_SIZE <= pad_size_regs;
    -------------------------------------------------------------------------------
    -- Quantized Convolution (strip) Use Threshold Register
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                use_th_regs <= (others => '0');
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                use_th_regs <= (others => '0');
            else
                for pos in use_th_regs'range loop
                    if (regs_load(pos+USE_TH_REGS_LO) = '1') then
                        use_th_regs(pos) <= regs_wbit(pos+USE_TH_REGS_LO);
                    end if;
                end loop;
            end if;
        end if;
    end process;
    regs_rbit(USE_TH_REGS_HI downto USE_TH_REGS_LO) <= use_th_regs;
    regs_rbit(USE_TH_RESV_HI downto USE_TH_RESV_LO) <= (USE_TH_RESV_HI downto USE_TH_RESV_LO => '0');
    USE_TH <= use_th_regs(0);
    -------------------------------------------------------------------------------
    -- Interrupt Request
    -------------------------------------------------------------------------------
    process (CLK, RST) begin
        if (RST = '1') then
                IRQ <= '0';
        elsif (CLK'event and CLK = '1') then
            if (CLR = '1') then
                IRQ <= '0';
            elsif (irq_enable = '1' and status_irq = '1') then
                IRQ <= '1';
            else
                IRQ <= '0';
            end if;
        end if;
    end process;
end RTL;
-----------------------------------------------------------------------------------
--!     @file    qconv_strip_th_data_axi_reader.vhd
--!     @brief   Quantized Convolution (strip) Thresholds Data AXI Reader Module
--!     @version 0.1.0
--!     @date    2019/4/26
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_TH_DATA_AXI_READER is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        QCONV_PARAM     : --! @brief QCONV PARAMETER :
                          QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
        AXI_ADDR_WIDTH  : --! @brief AXI ADDRESS WIDTH :
                          integer range 1 to   64 := 32;
        AXI_DATA_WIDTH  : --! @brief AXI DATA WIDTH :
                          integer range 8 to 1024 := 64;
        AXI_ID_WIDTH    : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_USER_WIDTH  : --! @brief AXI ID WIDTH :
                          integer := 8;
        AXI_XFER_SIZE   : --! @brief AXI MAX XFER_SIZE :
                          integer := 128*(64/8);
        AXI_ID          : --! @brief AXI ID :
                          integer := 0;
        AXI_PROT        : --! @brief AXI PROT :
                          integer := 1;
        AXI_QOS         : --! @brief AXI QOS :
                          integer := 0;
        AXI_REGION      : --! @brief AXI REGION :
                          integer := 0;
        AXI_CACHE       : --! @brief AXI REGION :
                          integer := 15;
        AXI_REQ_QUEUE   : --! @brief AXI REQUEST QUEUE SIZE :
                          integer := 4;
        REQ_ADDR_WIDTH  : --! @brief REQUEST ADDRESS WIDTH :
                          integer := 32
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        CLK             : in  std_logic;
        RST             : in  std_logic;
        CLR             : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        AXI_ARID        : out std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_ARADDR      : out std_logic_vector(AXI_ADDR_WIDTH  -1 downto 0);
        AXI_ARLEN       : out std_logic_vector(7 downto 0);
        AXI_ARSIZE      : out std_logic_vector(2 downto 0);
        AXI_ARBURST     : out std_logic_vector(1 downto 0);
        AXI_ARLOCK      : out std_logic_vector(0 downto 0);
        AXI_ARCACHE     : out std_logic_vector(3 downto 0);
        AXI_ARPROT      : out std_logic_vector(2 downto 0);
        AXI_ARQOS       : out std_logic_vector(3 downto 0);
        AXI_ARREGION    : out std_logic_vector(3 downto 0);
        AXI_ARUSER      : out std_logic_vector(AXI_USER_WIDTH  -1 downto 0);
        AXI_ARVALID     : out std_logic;
        AXI_ARREADY     : in  std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        AXI_RID         : in  std_logic_vector(AXI_ID_WIDTH    -1 downto 0);
        AXI_RDATA       : in  std_logic_vector(AXI_DATA_WIDTH  -1 downto 0);
        AXI_RRESP       : in  std_logic_vector(1 downto 0);
        AXI_RLAST       : in  std_logic;
        AXI_RVALID      : in  std_logic;
        AXI_RREADY      : out std_logic;
    -------------------------------------------------------------------------------
    -- AXI4 Stream Master Interface.
    -------------------------------------------------------------------------------
        O_DATA          : out std_logic_vector(QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS-1 downto 0);
        O_LAST          : out std_logic;
        O_VALID         : out std_logic;
        O_READY         : in  std_logic;
    -------------------------------------------------------------------------------
    -- Request / Response Interface.
    -------------------------------------------------------------------------------
        REQ_VALID       : in  std_logic;
        REQ_ADDR        : in  std_logic_vector(REQ_ADDR_WIDTH -1 downto 0);
        REQ_OUT_C       : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_OUT_C_POS   : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_OUT_C_SIZE  : in  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
        REQ_READY       : out std_logic;
        RES_VALID       : out std_logic;
        RES_NONE        : out std_logic;
        RES_ERROR       : out std_logic;
        RES_READY       : in  std_logic
    );
end QCONV_STRIP_TH_DATA_AXI_READER;
-----------------------------------------------------------------------------------
-- アーキテクチャ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.AXI4_TYPES.all;
use     PIPEWORK.AXI4_COMPONENTS.AXI4_MASTER_READ_INTERFACE;
use     PIPEWORK.PUMP_COMPONENTS.PUMP_STREAM_INTAKE_CONTROLLER;
use     PIPEWORK.IMAGE_TYPES.all;
use     PIPEWORK.IMAGE_COMPONENTS.IMAGE_SLICE_MASTER_CONTROLLER;
use     PIPEWORK.COMPONENTS.SDPRAM;
architecture RTL of QCONV_STRIP_TH_DATA_AXI_READER is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MAX(A,B: integer) return integer is
    begin
        if (A > B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B: integer) return integer is
    begin
        if (A < B) then return A;
        else            return B;
        end if;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function  MIN(A,B,C: integer) return integer is
    begin
        return MIN(A,MIN(B,C));
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    function CALC_BITS(SIZE:integer) return integer is
        variable bits : integer;
    begin
        bits := 0;
        while (2**bits < SIZE) loop
            bits := bits + 1;
        end loop;
        return bits;
    end function;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  IMAGE_SHAPE           :  IMAGE_SHAPE_TYPE := NEW_IMAGE_SHAPE(
                                           ELEM_BITS => QCONV_PARAM.NBITS_OUT_DATA * QCONV_PARAM.NUM_THRESHOLDS,
                                           C         => NEW_IMAGE_SHAPE_SIDE_EXTERNAL(QCONV_PARAM.MAX_OUT_C),
                                           X         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1),
                                           Y         => NEW_IMAGE_SHAPE_SIDE_CONSTANT(1)
                                       );
    signal    req_image_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_slice_c_pos       :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_slice_c_size      :  integer range 0 to IMAGE_SHAPE.C.MAX_SIZE;
    signal    req_axi_addr          :  std_logic_vector(AXI_ADDR_WIDTH-1 downto 0);
    -------------------------------------------------------------------------------
    -- 一回のトランザクションで転送する最大転送バイト数
    -------------------------------------------------------------------------------
    constant  MAX_XFER_BYTES        :  integer := MIN(4096, 256*(AXI_DATA_WIDTH/8), 2**AXI_XFER_SIZE);
    constant  MAX_XFER_SIZE         :  integer := CALC_BITS(MAX_XFER_BYTES);
    ------------------------------------------------------------------------------
    -- バッファの容量をバイト数で示す.
    ------------------------------------------------------------------------------
    constant  BUF_BYTES             :  integer := MAX_XFER_BYTES*2;
    ------------------------------------------------------------------------------
    -- バッファの容量(バイト数)を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DEPTH             :  integer := CALC_BITS(BUF_BYTES);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を示す.
    ------------------------------------------------------------------------------
    constant  BUF_WIDTH             :  integer := MAX(AXI_DATA_WIDTH, O_DATA'length);
    ------------------------------------------------------------------------------
    -- バッファのデータ幅のビット数を２のべき乗値で示す.
    ------------------------------------------------------------------------------
    constant  BUF_DATA_BIT_SIZE     :  integer := CALC_BITS(BUF_WIDTH);
    ------------------------------------------------------------------------------
    -- 入力側のフロー制御用定数.
    ------------------------------------------------------------------------------
    constant  I_FLOW_VALID          :  integer := 1;
    constant  I_USE_PUSH_BUF_SIZE   :  integer := 0;
    constant  I_FIXED_FLOW_OPEN     :  integer := 0;
    constant  I_FIXED_POOL_OPEN     :  integer := 1;
    constant  I_REQ_ADDR_VALID      :  integer := 1;
    constant  I_REQ_SIZE_VALID      :  integer := 1;
    constant  I_FLOW_READY_LEVEL    :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(BUF_BYTES - MAX_XFER_BYTES    , BUF_DEPTH+1));
    constant  I_BUF_READY_LEVEL     :  std_logic_vector(BUF_DEPTH downto 0)
                                    := std_logic_vector(to_unsigned(BUF_BYTES - 4*AXI_DATA_WIDTH/8, BUF_DEPTH+1));
    constant  I_MAX_REQ_SIZE        :  integer := IMAGE_SHAPE.X.MAX_SIZE * IMAGE_SHAPE.C.MAX_SIZE * IMAGE_SHAPE.ELEM_BITS / 8;
    constant  REQ_SIZE_WIDTH        :  integer := CALC_BITS(I_MAX_REQ_SIZE+1);
    -------------------------------------------------------------------------------
    -- AXI I/F 定数
    -------------------------------------------------------------------------------
    constant  AXI_REQ_PROT          :  AXI4_APROT_TYPE
                                    := std_logic_vector(to_unsigned(AXI_PROT  , AXI4_APROT_WIDTH  ));
    constant  AXI_REQ_QOS           :  AXI4_AQOS_TYPE
                                    := std_logic_vector(to_unsigned(AXI_QOS   , AXI4_AQOS_WIDTH   ));
    constant  AXI_REQ_REGION        :  AXI4_AREGION_TYPE
                                    := std_logic_vector(to_unsigned(AXI_REGION, AXI4_AREGION_WIDTH));
    constant  AXI_REQ_CACHE         :  AXI4_ACACHE_TYPE
                                    := std_logic_vector(to_unsigned(AXI_CACHE , AXI4_ACACHE_WIDTH ));
    constant  AXI_REQ_ID            :  std_logic_vector(AXI_ID_WIDTH -1 downto 0)
                                    := std_logic_vector(to_unsigned(AXI_ID    , AXI_ID_WIDTH      ));
    constant  AXI_REQ_LOCK          :  AXI4_ALOCK_TYPE  := (others => '0');
    constant  AXI_REQ_SPECULATIVE   :  std_logic := '1';
    constant  AXI_REQ_SAFETY        :  std_logic := '0';
    constant  AXI_ALIGNMENT_BITS    :  integer := 32;
    constant  AXI_ACK_REGS          :  integer := 1;
    constant  AXI_RDATA_REGS        :  integer := 3;
    constant  OPEN_INFO_BITS        :  integer := 4;
    constant  CLOSE_INFO_BITS       :  integer := 4;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_tran_start          :  std_logic;
    signal    i_tran_first          :  std_logic;
    signal    i_tran_last           :  std_logic;
    signal    i_tran_addr           :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_tran_addr_load      :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_tran_size           :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_tran_size_load      :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_tran_busy           :  std_logic;
    signal    i_tran_done           :  std_logic;
    signal    i_tran_error          :  std_logic;
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    i_req_valid           :  std_logic;
    signal    i_req_addr            :  std_logic_vector(AXI_ADDR_WIDTH -1 downto 0);
    signal    i_req_size            :  std_logic_vector(REQ_SIZE_WIDTH -1 downto 0);
    signal    i_req_buf_ptr         :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    i_req_first           :  std_logic;
    signal    i_req_last            :  std_logic;
    signal    i_req_ready           :  std_logic;
    signal    i_ack_valid           :  std_logic;
    signal    i_ack_size            :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_ack_error           :  std_logic;
    signal    i_ack_next            :  std_logic;
    signal    i_ack_last            :  std_logic;
    signal    i_ack_stop            :  std_logic;
    signal    i_ack_none            :  std_logic;
    signal    i_xfer_busy           :  std_logic;
    signal    i_xfer_done           :  std_logic;
    signal    i_xfer_error          :  std_logic;
    signal    i_flow_ready          :  std_logic;
    signal    i_flow_pause          :  std_logic;
    signal    i_flow_stop           :  std_logic;
    signal    i_flow_last           :  std_logic;
    signal    i_flow_size           :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_fin_valid      :  std_logic;
    signal    i_push_fin_last       :  std_logic;
    signal    i_push_fin_error      :  std_logic;
    signal    i_push_fin_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_rsv_valid      :  std_logic;
    signal    i_push_rsv_last       :  std_logic;
    signal    i_push_rsv_error      :  std_logic;
    signal    i_push_rsv_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_buf_reset      :  std_logic;
    signal    i_push_buf_valid      :  std_logic;
    signal    i_push_buf_last       :  std_logic;
    signal    i_push_buf_error      :  std_logic;
    signal    i_push_buf_size       :  std_logic_vector(BUF_DEPTH         downto 0);
    signal    i_push_buf_ready      :  std_logic;
    signal    i_open                :  std_logic;
    constant  i_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0) := (others => '0');
    constant  i_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0) := (others => '0');
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    buf_ren               :  std_logic;
    signal    buf_rptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_rdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_wen               :  std_logic;
    signal    buf_wptr              :  std_logic_vector(BUF_DEPTH      -1 downto 0);
    signal    buf_wdata             :  std_logic_vector(BUF_WIDTH      -1 downto 0);
    signal    buf_we                :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    signal    buf_ben               :  std_logic_vector(BUF_WIDTH/8    -1 downto 0);
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    signal    o_open                :  std_logic;
    signal    o_done                :  std_logic;
    signal    o_open_info           :  std_logic_vector(OPEN_INFO_BITS -1 downto 0);
    signal    o_open_valid          :  std_logic;
    signal    o_close_info          :  std_logic_vector(CLOSE_INFO_BITS-1 downto 0);
    signal    o_close_valid         :  std_logic;
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    req_image_c_size <= to_integer(to_01(unsigned(REQ_OUT_C     )));
    req_slice_c_pos  <= to_integer(to_01(unsigned(REQ_OUT_C_POS )));
    req_slice_c_size <= to_integer(to_01(unsigned(REQ_OUT_C_SIZE)));
    req_axi_addr     <= std_logic_vector(resize(unsigned(REQ_ADDR), AXI_ADDR_WIDTH));
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    MST_CTRL: IMAGE_SLICE_MASTER_CONTROLLER              -- 
        generic map (                                    -- 
            SOURCE_SHAPE        => IMAGE_SHAPE         , --
            SLICE_SHAPE         => IMAGE_SHAPE         , --
            MAX_SLICE_C_POS     => IMAGE_SHAPE.C.MAX_SIZE , --
            MAX_SLICE_X_POS     => 0                   , --
            MAX_SLICE_Y_POS     => 0                   , --
            ADDR_BITS           => AXI_ADDR_WIDTH      , --
            SIZE_BITS           => REQ_SIZE_WIDTH        --
        )                                                -- 
        port map (                                       -- 
        -------------------------------------------------------------------------------
        -- クロック&リセット信号
        -------------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            SOURCE_C_SIZE       => req_image_c_size    , -- In  :
            SLICE_C_POS         => req_slice_c_pos     , -- In  :
            SLICE_C_SIZE        => req_slice_c_size    , -- In  :
            REQ_ADDR            => req_axi_addr        , -- In  :
            REQ_VALID           => REQ_VALID           , -- In  :
            REQ_READY           => REQ_READY           , -- Out :
            RES_NONE            => RES_NONE            , -- Out :
            RES_ERROR           => RES_ERROR           , -- Out :
            RES_VALID           => RES_VALID           , -- Out :
            RES_READY           => RES_READY           , -- In  :
        -------------------------------------------------------------------------------
        -- 
        -------------------------------------------------------------------------------
            MST_ADDR            => i_tran_addr         , -- Out :
            MST_SIZE            => i_tran_size         , -- Out :
            MST_FIRST           => i_tran_first        , -- Out :
            MST_LAST            => i_tran_last         , -- Out :
            MST_START           => i_tran_start        , -- Out :
            MST_BUSY            => i_tran_busy         , -- In  :
            MST_DONE            => i_tran_done         , -- In  :
            MST_ERROR           => i_tran_error          -- In  :
        );                                               -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    i_tran_addr_load <= (others => '1') when (i_tran_start = '1') else (others => '0');
    i_tran_size_load <= (others => '1') when (i_tran_start = '1') else (others => '0');
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    PUMP_CTRL: PUMP_STREAM_INTAKE_CONTROLLER             -- 
        generic map (                                    -- 
            I_CLK_RATE          => 1                   , --
            I_REQ_ADDR_VALID    => I_REQ_ADDR_VALID    , --
            I_REQ_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            I_REG_ADDR_BITS     => AXI_ADDR_WIDTH      , --
            I_REQ_SIZE_VALID    => I_REQ_SIZE_VALID    , --
            I_REQ_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            I_REG_SIZE_BITS     => REQ_SIZE_WIDTH      , --
            I_REG_MODE_BITS     => 1                   , --
            I_REG_STAT_BITS     => 1                   , --
            I_USE_PUSH_BUF_SIZE => I_USE_PUSH_BUF_SIZE , --
            I_FIXED_FLOW_OPEN   => I_FIXED_FLOW_OPEN   , --
            I_FIXED_POOL_OPEN   => I_FIXED_POOL_OPEN   , --
            O_CLK_RATE          => 1                   , --
            O_DATA_BITS         => O_DATA'length       , --
            BUF_DEPTH           => BUF_DEPTH           , --
            BUF_DATA_BITS       => BUF_WIDTH           , --
            I2O_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            I2O_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            O2I_OPEN_INFO_BITS  => OPEN_INFO_BITS      , --
            O2I_CLOSE_INFO_BITS => CLOSE_INFO_BITS     , --
            I2O_DELAY_CYCLE     => 1                     --
        )                                                -- 
        port map (                                       -- 
        ---------------------------------------------------------------------------
        --Reset Signals.
        ---------------------------------------------------------------------------
            RST                 => RST                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Clock and Clock Enable.
        ---------------------------------------------------------------------------
            I_CLK               => CLK                 , --  In  :
            I_CLR               => CLR                 , --  In  :
            I_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Control Register Interface.
        ---------------------------------------------------------------------------
            I_ADDR_L            => i_tran_addr_load    , --  In  :
            I_ADDR_D            => i_tran_addr         , --  In  :
            I_SIZE_L            => i_tran_size_load    , --  In  :
            I_SIZE_D            => i_tran_size         , --  In  :
            I_START_L           => i_tran_start        , --  In  :
            I_START_D           => i_tran_start        , --  In  :
            I_FIRST_L           => i_tran_start        , --  In  :
            I_FIRST_D           => i_tran_first        , --  In  :
            I_LAST_L            => i_tran_start        , --  In  :
            I_LAST_D            => i_tran_last         , --  In  :
            I_DONE_EN_L         => i_tran_start        , --  In  :
            I_DONE_EN_D         => '0'                 , --  In  :
            I_DONE_ST_L         => i_tran_start        , --  In  :
            I_DONE_ST_D         => '0'                 , --  In  :
            I_ERR_ST_L          => i_tran_start        , --  In  :
            I_ERR_ST_D          => '0'                 , --  In  :
            I_CLOSE_ST_L        => i_tran_start        , --  In  :
            I_CLOSE_ST_D        => '0'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Configuration Signals.
        ---------------------------------------------------------------------------
            I_BUF_READY_LEVEL   => I_BUF_READY_LEVEL   , --  In  :
            I_FLOW_READY_LEVEL  => I_FLOW_READY_LEVEL  , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transaction Command Request Signals.
        ---------------------------------------------------------------------------
            I_REQ_VALID         => i_req_valid         , --  Out :
            I_REQ_ADDR          => i_req_addr          , --  Out :
            I_REQ_SIZE          => i_req_size          , --  Out :
            I_REQ_BUF_PTR       => i_req_buf_ptr       , --  Out :
            I_REQ_FIRST         => i_req_first         , --  Out :
            I_REQ_LAST          => i_req_last          , --  Out :
            I_REQ_READY         => i_req_ready         , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transaction Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            I_ACK_VALID         => i_ack_valid         , --  In  :
            I_ACK_SIZE          => i_ack_size          , --  In  :
            I_ACK_ERROR         => i_ack_error         , --  In  :
            I_ACK_NEXT          => i_ack_next          , --  In  :
            I_ACK_LAST          => i_ack_last          , --  In  :
            I_ACK_STOP          => i_ack_stop          , --  In  :
            I_ACK_NONE          => i_ack_none          , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Transfer Status Signals.
        ---------------------------------------------------------------------------
            I_XFER_BUSY         => i_xfer_busy         , --  In  :
            I_XFER_DONE         => i_xfer_done         , --  In  :
            I_XFER_ERROR        => i_xfer_error        , --  In  :
        ---------------------------------------------------------------------------
        -- Intake Flow Control Signals.
        ---------------------------------------------------------------------------
            I_FLOW_READY        => i_flow_ready        , --  Out :
            I_FLOW_PAUSE        => i_flow_pause        , --  Out :
            I_FLOW_STOP         => i_flow_stop         , --  Out :
            I_FLOW_LAST         => i_flow_last         , --  Out :
            I_FLOW_SIZE         => i_flow_size         , --  Out :
            I_PUSH_FIN_VALID    => i_push_fin_valid    , --  In  :
            I_PUSH_FIN_LAST     => i_push_fin_last     , --  In  :
            I_PUSH_FIN_ERROR    => i_push_fin_error    , --  In  :
            I_PUSH_FIN_SIZE     => i_push_fin_size     , --  In  :
            I_PUSH_RSV_VALID    => i_push_rsv_valid    , --  In  :
            I_PUSH_RSV_LAST     => i_push_rsv_last     , --  In  :
            I_PUSH_RSV_ERROR    => i_push_rsv_error    , --  In  :
            I_PUSH_RSV_SIZE     => i_push_rsv_size     , --  In  :
            I_PUSH_BUF_RESET    => i_push_buf_reset    , --  In  :
            I_PUSH_BUF_VALID    => i_push_buf_valid    , --  In  :
            I_PUSH_BUF_LAST     => i_push_buf_last     , --  In  :
            I_PUSH_BUF_ERROR    => i_push_buf_error    , --  In  :
            I_PUSH_BUF_SIZE     => i_push_buf_size     , --  In  :
            I_PUSH_BUF_READY    => i_push_buf_ready    , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Status.
        ---------------------------------------------------------------------------
            I_OPEN              => i_open              , --  Out :
            I_TRAN_BUSY         => i_tran_busy         , --  Out :
            I_TRAN_DONE         => i_tran_done         , --  Out :
            I_TRAN_ERROR        => i_tran_error        , --  Out :
        ---------------------------------------------------------------------------
        -- Intake Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            I_I2O_OPEN_INFO     => i_open_info         , --  In  :
            I_I2O_CLOSE_INFO    => i_close_info        , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Clock and Clock Enable.
        ---------------------------------------------------------------------------
            O_CLK               => CLK                 , --  In  :
            O_CLR               => CLR                 , --  In  :
            O_CKE               => '1'                 , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Stream Interface.
        ---------------------------------------------------------------------------
            O_DATA              => O_DATA              , --  Out :
            O_STRB              => open                , --  Out :
            O_LAST              => O_LAST              , --  Out :
            O_VALID             => O_VALID             , --  Out :
            O_READY             => O_READY             , --  In  :
        ---------------------------------------------------------------------------
        -- Outlet Open/Close Infomation Interface
        ---------------------------------------------------------------------------
            O_O2I_OPEN_INFO     => o_open_info         , --  In  :
            O_O2I_OPEN_VALID    => o_open_valid        , --  In  :
            O_O2I_CLOSE_INFO    => o_close_info        , --  In  :
            O_O2I_CLOSE_VALID   => o_close_valid       , --  In  :
            O_I2O_OPEN_INFO     => o_open_info         , --  Out :
            O_I2O_OPEN_VALID    => o_open_valid        , --  Out :
            O_I2O_CLOSE_INFO    => o_close_info        , --  Out :
            O_I2O_CLOSE_VALID   => o_close_valid       , --  Out :
        ---------------------------------------------------------------------------
        -- Outlet Buffer Read Interface.
        ---------------------------------------------------------------------------
            BUF_REN             => buf_ren             , --  Out :
            BUF_PTR             => buf_rptr            , --  Out :
            BUF_DATA            => buf_rdata             --  In  :
        );                                               --
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    AXI_IF: AXI4_MASTER_READ_INTERFACE                   -- 
        generic map (                                    -- 
            AXI4_ADDR_WIDTH     => AXI_ADDR_WIDTH      , -- 
            AXI4_DATA_WIDTH     => AXI_DATA_WIDTH      , --   
            AXI4_ID_WIDTH       => AXI_ID_WIDTH        , --   
            VAL_BITS            => 1                   , --   
            REQ_SIZE_BITS       => REQ_SIZE_WIDTH      , --   
            REQ_SIZE_VALID      => 1                   , --   
            FLOW_VALID          => I_FLOW_VALID        , --   
            BUF_DATA_WIDTH      => BUF_WIDTH           , --   
            BUF_PTR_BITS        => BUF_DEPTH           , --   
            ALIGNMENT_BITS      => AXI_ALIGNMENT_BITS  , --   
            XFER_SIZE_BITS      => BUF_DEPTH+1         , --   
            XFER_MIN_SIZE       => MAX_XFER_SIZE       , --   
            XFER_MAX_SIZE       => MAX_XFER_SIZE       , --   
            QUEUE_SIZE          => AXI_REQ_QUEUE       , --   
            RDATA_REGS          => AXI_RDATA_REGS      , --   
            ACK_REGS            => AXI_ACK_REGS          --   
        )                                                -- 
        port map(                                        --
        ---------------------------------------------------------------------------
        -- Clock and Reset Signals.
        ---------------------------------------------------------------------------
            CLK                 => CLK                 , -- In  :
            RST                 => RST                 , -- In  :
            CLR                 => CLR                 , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            ARID                => AXI_ARID            , -- Out :
            ARADDR              => AXI_ARADDR          , -- Out :
            ARLEN               => AXI_ARLEN           , -- Out :
            ARSIZE              => AXI_ARSIZE          , -- Out :
            ARBURST             => AXI_ARBURST         , -- Out :
            ARLOCK              => AXI_ARLOCK          , -- Out :
            ARCACHE             => AXI_ARCACHE         , -- Out :
            ARPROT              => AXI_ARPROT          , -- Out :
            ARQOS               => AXI_ARQOS           , -- Out :
            ARREGION            => AXI_ARREGION        , -- Out :
            ARVALID             => AXI_ARVALID         , -- Out :
            ARREADY             => AXI_ARREADY         , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            RID                 => AXI_RID             , -- In  :
            RDATA               => AXI_RDATA           , -- In  :
            RRESP               => AXI_RRESP           , -- In  :
            RLAST               => AXI_RLAST           , -- In  :
            RVALID              => AXI_RVALID          , -- In  :
            RREADY              => AXI_RREADY          , -- Out :
        ---------------------------------------------------------------------------
        -- Command Request Signals.
        ---------------------------------------------------------------------------
            XFER_SIZE_SEL       => "1"                 , -- In  :
            REQ_ADDR            => i_req_addr          , -- In  :
            REQ_SIZE            => i_req_size          , -- In  :
            REQ_ID              => AXI_REQ_ID          , -- In  :
            REQ_BURST           => AXI4_ABURST_INCR    , -- In  :
            REQ_LOCK            => AXI_REQ_LOCK        , -- In  :
            REQ_CACHE           => AXI_REQ_CACHE       , -- In  :
            REQ_PROT            => AXI_REQ_PROT        , -- In  :
            REQ_QOS             => AXI_REQ_QOS         , -- In  :
            REQ_REGION          => AXI_REQ_REGION      , -- In  :
            REQ_BUF_PTR         => i_req_buf_ptr       , -- In  :
            REQ_FIRST           => i_req_first         , -- In  :
            REQ_LAST            => i_req_last          , -- In  :
            REQ_SPECULATIVE     => AXI_REQ_SPECULATIVE , -- In  :
            REQ_SAFETY          => AXI_REQ_SAFETY      , -- In  :
            REQ_VAL(0)          => i_req_valid         , -- In  :
            REQ_RDY             => i_req_ready         , -- Out :
        ---------------------------------------------------------------------------
        -- Command Acknowledge Signals.
        ---------------------------------------------------------------------------
            ACK_VAL(0)          => i_ack_valid         , -- Out :
            ACK_NEXT            => i_ack_next          , -- Out :
            ACK_LAST            => i_ack_last          , -- Out :
            ACK_ERROR           => i_ack_error         , -- Out :
            ACK_STOP            => i_ack_stop          , -- Out :
            ACK_NONE            => i_ack_none          , -- Out :
            ACK_SIZE            => i_ack_size          , -- Out :
        ---------------------------------------------------------------------------
        -- Transfer Status Signal.
        ---------------------------------------------------------------------------
            XFER_BUSY(0)        => i_xfer_busy         , -- Out :
            XFER_ERROR(0)       => i_xfer_error        , -- Out :
            XFER_DONE(0)        => i_xfer_done         , -- Out :
        ---------------------------------------------------------------------------
        -- Flow Control Signals.
        ---------------------------------------------------------------------------
            FLOW_STOP           => i_flow_stop         , -- In  :
            FLOW_PAUSE          => i_flow_pause        , -- In  :
            FLOW_LAST           => i_flow_last         , -- In  :
            FLOW_SIZE           => i_flow_size         , -- In  :
        ---------------------------------------------------------------------------
        -- Push Reserve Size Signals.
        ---------------------------------------------------------------------------
            PUSH_RSV_VAL(0)     => i_push_rsv_valid    , -- Out :
            PUSH_RSV_LAST       => i_push_rsv_last     , -- Out :
            PUSH_RSV_ERROR      => i_push_rsv_error    , -- Out :
            PUSH_RSV_SIZE       => i_push_rsv_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Push Final Size Signals.
        ---------------------------------------------------------------------------
            PUSH_FIN_VAL(0)     => i_push_fin_valid    , -- Out :
            PUSH_FIN_LAST       => i_push_fin_last     , -- Out :
            PUSH_FIN_ERROR      => i_push_fin_error    , -- Out :
            PUSH_FIN_SIZE       => i_push_fin_size     , -- Out :
        ---------------------------------------------------------------------------
        -- Push Buffer Size Signals.
        ---------------------------------------------------------------------------
            PUSH_BUF_RESET(0)   => i_push_buf_reset    , -- Out :
            PUSH_BUF_VAL(0)     => i_push_buf_valid    , -- Out :
            PUSH_BUF_LAST       => i_push_buf_last     , -- Out :
            PUSH_BUF_ERROR      => i_push_buf_error    , -- Out :
            PUSH_BUF_SIZE       => i_push_buf_size     , -- Out :
            PUSH_BUF_RDY(0)     => i_push_buf_ready    , -- In  :
        ---------------------------------------------------------------------------
        -- Read Buffer Interface Signals.
        ---------------------------------------------------------------------------
            BUF_WEN(0)          => buf_wen             , -- Out :
            BUF_BEN             => buf_ben             , -- Out :
            BUF_DATA            => buf_wdata           , -- Out :
            BUF_PTR             => buf_wptr              -- Out :
        );                                               -- 
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    RAM: SDPRAM 
        generic map(
            DEPTH       => BUF_DEPTH+3         ,
            RWIDTH      => BUF_DATA_BIT_SIZE   , --
            WWIDTH      => BUF_DATA_BIT_SIZE   , --
            WEBIT       => BUF_DATA_BIT_SIZE-3 , --
            ID          => 0                     -- 
        )                                        -- 
        port map (                               -- 
            WCLK        => CLK                 , -- In  :
            WE          => buf_we              , -- In  :
            WADDR       => buf_wptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            WDATA       => buf_wdata           , -- In  :
            RCLK        => CLK                 , -- In  :
            RADDR       => buf_rptr(BUF_DEPTH-1 downto BUF_DATA_BIT_SIZE-3), -- In  :
            RDATA       => buf_rdata             -- Out :
        );
    buf_we <= buf_ben when (buf_wen = '1') else (others => '0');
end RTL;

-----------------------------------------------------------------------------------
--!     @file    qconv_strip_axi_core.vhd
--!     @brief   Quantized Convolution (strip) AXI I/F Core Module
--!     @version 0.1.0
--!     @date    2019/4/27
--!     @author  Ichiro Kawazome <ichiro_k@ca2.so-net.ne.jp>
-----------------------------------------------------------------------------------
--
--      Copyright (C) 2018-2019 Ichiro Kawazome
--      All rights reserved.
--
--      Redistribution and use in source and binary forms, with or without
--      modification, are permitted provided that the following conditions
--      are met:
--
--        1. Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
--
--        2. Redistributions in binary form must reproduce the above copyright
--           notice, this list of conditions and the following disclaimer in
--           the documentation and/or other materials provided with the
--           distribution.
--
--      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
--      A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
--      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
--      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
--      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
--      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
--      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
--      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
--      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
-----------------------------------------------------------------------------------
--! @brief 
-----------------------------------------------------------------------------------
entity  QCONV_STRIP_AXI_CORE is
    -------------------------------------------------------------------------------
    -- 
    -------------------------------------------------------------------------------
    generic (
        ID                  : --! @brief QCONV ID STRING :
                              string(1 to 8) := "QCONV-S1";
        IN_BUF_SIZE         : --! @brief IN DATA BUFFER SIZE :
                              --! 入力バッファの容量を指定する.
                              --! * ここで指定する単位は1ワード単位.
                              --! * 1ワードは QCONV_PARAM.NBITS_IN_DATA * QCONV_PARAM.NBITS_PER_WORD
                              --!   = 64 bit.
                              --! * 入力バッファの容量は 入力チャネル × イメージの幅.
                              integer := 512*4*1;  -- 512word × BANK_SIZE × IN_C_UNROLL 
        K_BUF_SIZE          : --! @brief K DATA BUFFER SIZE :
                              --! カーネル係数バッファの容量を指定する.
                              --! * ここで指定する単位は1ワード単位.
                              --! * 1ワードは 3 * 3 * QCONV_PARAM.NBITS_K_DATA * QCONV_PARAM.NBITS_PER_WORD
                              --! * カーネル係数バッファの容量は K_BUF_SIZE * 288bit になる.
                              integer := 512*3*3*8*1;  -- 512word × 3 × 3 × OUT_C_UNROLL × IN_C_UNROLL
        TH_BUF_SIZE         : --! @brief THRESHOLDS DATA BUFFER SIZE :
                              --! THRESHOLDS バッファの容量を指定する.
                              --! * ここで指定する単位は1ワード単位.
                              --! * 1ワードは QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS
                              --! * = 64bit
                              integer := 512*8;
        IN_C_UNROLL         : --! @brief INPUT  CHANNEL UNROLL SIZE :
                              integer := 1;
        OUT_C_UNROLL        : --! @brief OUTPUT CHANNEL UNROLL SIZE :
                              integer := 8;
        DATA_ADDR_WIDTH     : --! @brief DATA ADDRESS WIDTH :
                              --! IN_DATA/OUT_DATA/K_DATA/TH_DATA のメモリアドレスのビット幅を指定する.
                              integer := 32;
        S_AXI_ADDR_WIDTH    : --! @brief CSR I/F AXI ADDRRESS WIDTH :
                              integer := 32;
        S_AXI_DATA_WIDTH    : --! @brief CSR I/F AXI DATA WIDTH :
                              integer := 32;
        S_AXI_ID_WIDTH      : --! @brief CSR I/F AXI4 ID WIDTH :
                              integer := 4;
        I_AXI_ADDR_WIDTH    : --! @brief IN  DATA AXI ADDRESS WIDTH :
                              integer := 32;
        I_AXI_DATA_WIDTH    : --! @brief IN  DATA AXI DATA WIDTH :
                              integer := 64;
        I_AXI_ID_WIDTH      : --! @brief IN  DATA AXI ID WIDTH :
                              integer := 8;
        I_AXI_USER_WIDTH    : --! @brief IN  DATA AXI ID WIDTH :
                              integer := 8;
        I_AXI_XFER_SIZE     : --! @brief IN  DATA AXI MAX XFER_SIZE :
                              integer := 11;
        I_AXI_ID            : --! @brief IN  DATA AXI ID :
                              integer := 0;
        I_AXI_PROT          : --! @brief IN  DATA AXI PROT :
                              integer := 1;
        I_AXI_QOS           : --! @brief IN  DATA AXI QOS :
                              integer := 0;
        I_AXI_REGION        : --! @brief IN  DATA AXI REGION :
                              integer := 0;
        I_AXI_CACHE         : --! @brief IN  DATA AXI REGION :
                              integer := 15;
        I_AXI_REQ_QUEUE     : --! @brief IN  DATA AXI REQUEST QUEUE SIZE :
                              integer := 4;
        O_AXI_ADDR_WIDTH    : --! @brief OUT DATA AXI ADDRESS WIDTH :
                              integer := 32;
        O_AXI_DATA_WIDTH    : --! @brief OUT DATA AXI DATA WIDTH :
                              integer := 64;
        O_AXI_ID_WIDTH      : --! @brief OUT DATA AXI ID WIDTH :
                              integer := 8;
        O_AXI_USER_WIDTH    : --! @brief OUT DATA AXI ID WIDTH :
                              integer := 8;
        O_AXI_XFER_SIZE     : --! @brief OUT DATA AXI MAX XFER_SIZE :
                              integer := 11;
        O_AXI_ID            : --! @brief OUT DATA AXI ID :
                              integer := 0;
        O_AXI_PROT          : --! @brief OUT DATA AXI PROT :
                              integer := 1;
        O_AXI_QOS           : --! @brief OUT DATA AXI QOS :
                              integer := 0;
        O_AXI_REGION        : --! @brief OUT DATA AXI REGION :
                              integer := 0;
        O_AXI_CACHE         : --! @brief OUT DATA AXI REGION :
                              integer := 15;
        O_AXI_REQ_QUEUE     : --! @brief OUT DATA AXI REQUEST QUEUE SIZE :
                              integer := 4;
        K_AXI_ADDR_WIDTH    : --! @brief K   DATA AXI ADDRESS WIDTH :
                              integer := 32;
        K_AXI_DATA_WIDTH    : --! @brief K   DATA AXI DATA WIDTH :
                              integer := 64;
        K_AXI_ID_WIDTH      : --! @brief K   DATA AXI ID WIDTH :
                              integer := 8;
        K_AXI_USER_WIDTH    : --! @brief K   DATA AXI ID WIDTH :
                              integer := 8;
        K_AXI_XFER_SIZE     : --! @brief K   DATA AXI MAX XFER_SIZE :
                              integer := 11;
        K_AXI_ID            : --! @brief K   DATA AXI ID :
                              integer := 0;
        K_AXI_PROT          : --! @brief K   DATA AXI PROT :
                              integer := 1;
        K_AXI_QOS           : --! @brief K   DATA AXI QOS :
                              integer := 0;
        K_AXI_REGION        : --! @brief K   DATA AXI REGION :
                              integer := 0;
        K_AXI_CACHE         : --! @brief K   DATA AXI REGION :
                              integer := 15;
        K_AXI_REQ_QUEUE     : --! @brief K   DATA AXI REQUEST QUEUE SIZE :
                              integer := 4;
        T_AXI_ADDR_WIDTH    : --! @brief TH  DATA AXI ADDRESS WIDTH :
                              integer := 32;
        T_AXI_DATA_WIDTH    : --! @brief TH  DATA AXI DATA WIDTH :
                              integer := 64;
        T_AXI_ID_WIDTH      : --! @brief TH  DATA AXI ID WIDTH :
                              integer := 8;
        T_AXI_USER_WIDTH    : --! @brief TH  DATA AXI ID WIDTH :
                              integer := 8;
        T_AXI_XFER_SIZE     : --! @brief TH  DATA AXI MAX XFER_SIZE :
                              integer := 11;
        T_AXI_ID            : --! @brief TH  DATA AXI ID :
                              integer := 0;
        T_AXI_PROT          : --! @brief TH  DATA AXI PROT :
                              integer := 1;
        T_AXI_QOS           : --! @brief TH  DATA AXI QOS :
                              integer := 0;
        T_AXI_REGION        : --! @brief TH  DATA AXI REGION :
                              integer := 0;
        T_AXI_CACHE         : --! @brief TH  DATA AXI REGION :
                              integer := 15;
        T_AXI_REQ_QUEUE     : --! @brief TH  DATA AXI REQUEST QUEUE SIZE :
                              integer := 1
    );
    port(
    -------------------------------------------------------------------------------
    -- Clock / Reset Signals.
    -------------------------------------------------------------------------------
        ACLK                : in  std_logic;
        ARESETn             : in  std_logic;
    -------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        S_AXI_ARID          : in  std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_ARADDR        : in  std_logic_vector(S_AXI_ADDR_WIDTH  -1 downto 0);
        S_AXI_ARLEN         : in  std_logic_vector(7 downto 0);
        S_AXI_ARSIZE        : in  std_logic_vector(2 downto 0);
        S_AXI_ARBURST       : in  std_logic_vector(1 downto 0);
        S_AXI_ARVALID       : in  std_logic;
        S_AXI_ARREADY       : out std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Read Data Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_RID           : out std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_RDATA         : out std_logic_vector(S_AXI_DATA_WIDTH  -1 downto 0);
        S_AXI_RRESP         : out std_logic_vector(1 downto 0);  
        S_AXI_RLAST         : out std_logic;
        S_AXI_RVALID        : out std_logic;
        S_AXI_RREADY        : in  std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Write Address Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_AWID          : in  std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_AWADDR        : in  std_logic_vector(S_AXI_ADDR_WIDTH  -1 downto 0);
        S_AXI_AWLEN         : in  std_logic_vector(7 downto 0);
        S_AXI_AWSIZE        : in  std_logic_vector(2 downto 0);
        S_AXI_AWBURST       : in  std_logic_vector(1 downto 0);
        S_AXI_AWVALID       : in  std_logic;
        S_AXI_AWREADY       : out std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Write Data Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_WDATA         : in  std_logic_vector(S_AXI_DATA_WIDTH  -1 downto 0);
        S_AXI_WSTRB         : in  std_logic_vector(S_AXI_DATA_WIDTH/8-1 downto 0);
        S_AXI_WLAST         : in  std_logic;
        S_AXI_WVALID        : in  std_logic;
        S_AXI_WREADY        : out std_logic;
    ------------------------------------------------------------------------------
    -- Control Status Register I/F AXI4 Write Response Channel Signals.
    ------------------------------------------------------------------------------
        S_AXI_BID           : out std_logic_vector(S_AXI_ID_WIDTH    -1 downto 0);
        S_AXI_BRESP         : out std_logic_vector(1 downto 0);
        S_AXI_BVALID        : out std_logic;
        S_AXI_BREADY        : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_ARID         : out std_logic_vector(I_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_ARADDR       : out std_logic_vector(I_AXI_ADDR_WIDTH  -1 downto 0);
        IO_AXI_ARLEN        : out std_logic_vector(7 downto 0);
        IO_AXI_ARSIZE       : out std_logic_vector(2 downto 0);
        IO_AXI_ARBURST      : out std_logic_vector(1 downto 0);
        IO_AXI_ARLOCK       : out std_logic_vector(0 downto 0);
        IO_AXI_ARCACHE      : out std_logic_vector(3 downto 0);
        IO_AXI_ARPROT       : out std_logic_vector(2 downto 0);
        IO_AXI_ARQOS        : out std_logic_vector(3 downto 0);
        IO_AXI_ARREGION     : out std_logic_vector(3 downto 0);
        IO_AXI_ARUSER       : out std_logic_vector(I_AXI_USER_WIDTH  -1 downto 0);
        IO_AXI_ARVALID      : out std_logic;
        IO_AXI_ARREADY      : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_RID          : in  std_logic_vector(I_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_RDATA        : in  std_logic_vector(I_AXI_DATA_WIDTH  -1 downto 0);
        IO_AXI_RRESP        : in  std_logic_vector(1 downto 0);
        IO_AXI_RLAST        : in  std_logic;
        IO_AXI_RVALID       : in  std_logic;
        IO_AXI_RREADY       : out std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_AWID         : out std_logic_vector(O_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_AWADDR       : out std_logic_vector(O_AXI_ADDR_WIDTH  -1 downto 0);
        IO_AXI_AWLEN        : out std_logic_vector(7 downto 0);
        IO_AXI_AWSIZE       : out std_logic_vector(2 downto 0);
        IO_AXI_AWBURST      : out std_logic_vector(1 downto 0);
        IO_AXI_AWLOCK       : out std_logic_vector(0 downto 0);
        IO_AXI_AWCACHE      : out std_logic_vector(3 downto 0);
        IO_AXI_AWPROT       : out std_logic_vector(2 downto 0);
        IO_AXI_AWQOS        : out std_logic_vector(3 downto 0);
        IO_AXI_AWREGION     : out std_logic_vector(3 downto 0);
        IO_AXI_AWUSER       : out std_logic_vector(O_AXI_USER_WIDTH  -1 downto 0);
        IO_AXI_AWVALID      : out std_logic;
        IO_AXI_AWREADY      : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_WID          : out std_logic_vector(O_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_WDATA        : out std_logic_vector(O_AXI_DATA_WIDTH  -1 downto 0);
        IO_AXI_WSTRB        : out std_logic_vector(O_AXI_DATA_WIDTH/8-1 downto 0);
        IO_AXI_WLAST        : out std_logic;
        IO_AXI_WVALID       : out std_logic;
        IO_AXI_WREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- IN/OUT DATA AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        IO_AXI_BID          : in  std_logic_vector(O_AXI_ID_WIDTH    -1 downto 0);
        IO_AXI_BRESP        : in  std_logic_vector(1 downto 0);
        IO_AXI_BVALID       : in  std_logic;
        IO_AXI_BREADY       : out std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_ARID          : out std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_ARADDR        : out std_logic_vector(K_AXI_ADDR_WIDTH  -1 downto 0);
        K_AXI_ARLEN         : out std_logic_vector(7 downto 0);
        K_AXI_ARSIZE        : out std_logic_vector(2 downto 0);
        K_AXI_ARBURST       : out std_logic_vector(1 downto 0);
        K_AXI_ARLOCK        : out std_logic_vector(0 downto 0);
        K_AXI_ARCACHE       : out std_logic_vector(3 downto 0);
        K_AXI_ARPROT        : out std_logic_vector(2 downto 0);
        K_AXI_ARQOS         : out std_logic_vector(3 downto 0);
        K_AXI_ARREGION      : out std_logic_vector(3 downto 0);
        K_AXI_ARUSER        : out std_logic_vector(K_AXI_USER_WIDTH  -1 downto 0);
        K_AXI_ARVALID       : out std_logic;
        K_AXI_ARREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_RID           : in  std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_RDATA         : in  std_logic_vector(K_AXI_DATA_WIDTH  -1 downto 0);
        K_AXI_RRESP         : in  std_logic_vector(1 downto 0);
        K_AXI_RLAST         : in  std_logic;
        K_AXI_RVALID        : in  std_logic;
        K_AXI_RREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_AWID          : out std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_AWADDR        : out std_logic_vector(K_AXI_ADDR_WIDTH  -1 downto 0);
        K_AXI_AWLEN         : out std_logic_vector(7 downto 0);
        K_AXI_AWSIZE        : out std_logic_vector(2 downto 0);
        K_AXI_AWBURST       : out std_logic_vector(1 downto 0);
        K_AXI_AWLOCK        : out std_logic_vector(0 downto 0);
        K_AXI_AWCACHE       : out std_logic_vector(3 downto 0);
        K_AXI_AWPROT        : out std_logic_vector(2 downto 0);
        K_AXI_AWQOS         : out std_logic_vector(3 downto 0);
        K_AXI_AWREGION      : out std_logic_vector(3 downto 0);
        K_AXI_AWUSER        : out std_logic_vector(K_AXI_USER_WIDTH  -1 downto 0);
        K_AXI_AWVALID       : out std_logic;
        K_AXI_AWREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_WID           : out std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_WDATA         : out std_logic_vector(K_AXI_DATA_WIDTH  -1 downto 0);
        K_AXI_WSTRB         : out std_logic_vector(K_AXI_DATA_WIDTH/8-1 downto 0);
        K_AXI_WLAST         : out std_logic;
        K_AXI_WVALID        : out std_logic;
        K_AXI_WREADY        : in  std_logic;
    -------------------------------------------------------------------------------
    -- K DATA AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        K_AXI_BID           : in  std_logic_vector(K_AXI_ID_WIDTH    -1 downto 0);
        K_AXI_BRESP         : in  std_logic_vector(1 downto 0);
        K_AXI_BVALID        : in  std_logic;
        K_AXI_BREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Read Address Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_ARID          : out std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_ARADDR        : out std_logic_vector(T_AXI_ADDR_WIDTH  -1 downto 0);
        T_AXI_ARLEN         : out std_logic_vector(7 downto 0);
        T_AXI_ARSIZE        : out std_logic_vector(2 downto 0);
        T_AXI_ARBURST       : out std_logic_vector(1 downto 0);
        T_AXI_ARLOCK        : out std_logic_vector(0 downto 0);
        T_AXI_ARCACHE       : out std_logic_vector(3 downto 0);
        T_AXI_ARPROT        : out std_logic_vector(2 downto 0);
        T_AXI_ARQOS         : out std_logic_vector(3 downto 0);
        T_AXI_ARREGION      : out std_logic_vector(3 downto 0);
        T_AXI_ARUSER        : out std_logic_vector(K_AXI_USER_WIDTH  -1 downto 0);
        T_AXI_ARVALID       : out std_logic;
        T_AXI_ARREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Read Data Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_RID           : in  std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_RDATA         : in  std_logic_vector(T_AXI_DATA_WIDTH  -1 downto 0);
        T_AXI_RRESP         : in  std_logic_vector(1 downto 0);
        T_AXI_RLAST         : in  std_logic;
        T_AXI_RVALID        : in  std_logic;
        T_AXI_RREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Write Address Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_AWID          : out std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_AWADDR        : out std_logic_vector(T_AXI_ADDR_WIDTH  -1 downto 0);
        T_AXI_AWLEN         : out std_logic_vector(7 downto 0);
        T_AXI_AWSIZE        : out std_logic_vector(2 downto 0);
        T_AXI_AWBURST       : out std_logic_vector(1 downto 0);
        T_AXI_AWLOCK        : out std_logic_vector(0 downto 0);
        T_AXI_AWCACHE       : out std_logic_vector(3 downto 0);
        T_AXI_AWPROT        : out std_logic_vector(2 downto 0);
        T_AXI_AWQOS         : out std_logic_vector(3 downto 0);
        T_AXI_AWREGION      : out std_logic_vector(3 downto 0);
        T_AXI_AWUSER        : out std_logic_vector(T_AXI_USER_WIDTH  -1 downto 0);
        T_AXI_AWVALID       : out std_logic;
        T_AXI_AWREADY       : in  std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Write Data Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_WID           : out std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_WDATA         : out std_logic_vector(T_AXI_DATA_WIDTH  -1 downto 0);
        T_AXI_WSTRB         : out std_logic_vector(T_AXI_DATA_WIDTH/8-1 downto 0);
        T_AXI_WLAST         : out std_logic;
        T_AXI_WVALID        : out std_logic;
        T_AXI_WREADY        : in  std_logic;
    -------------------------------------------------------------------------------
    -- TH DATA AXI4 Write Response Channel Signals.
    -------------------------------------------------------------------------------
        T_AXI_BID           : in  std_logic_vector(T_AXI_ID_WIDTH    -1 downto 0);
        T_AXI_BRESP         : in  std_logic_vector(1 downto 0);
        T_AXI_BVALID        : in  std_logic;
        T_AXI_BREADY        : out std_logic;
    -------------------------------------------------------------------------------
    -- Interrupt Request
    -------------------------------------------------------------------------------
        IRQ                 : out std_logic
    );
end  QCONV_STRIP_AXI_CORE;
-----------------------------------------------------------------------------------
-- アーキテクチャ本体
-----------------------------------------------------------------------------------
library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
library PIPEWORK;
use     PIPEWORK.AXI4_COMPONENTS.AXI4_REGISTER_INTERFACE;
library QCONV;
use     QCONV.QCONV_PARAMS.all;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_IN_DATA_AXI_READER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_K_DATA_AXI_READER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_TH_DATA_AXI_READER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_OUT_DATA_AXI_WRITER;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_CORE;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_REGISTERS;
use     QCONV.QCONV_COMPONENTS.QCONV_STRIP_CONTROLLER;
architecture RTL of QCONV_STRIP_AXI_CORE is
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  QCONV_PARAM           :  QCONV_PARAMS_TYPE := QCONV_COMMON_PARAMS;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    RESET                 :  std_logic;
    constant  CLEAR                 :  std_logic := '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  REGS_ADDR_WIDTH       :  integer :=  8;
    constant  REGS_DATA_WIDTH       :  integer := 64;
    signal    regs_req              :  std_logic;
    signal    regs_write            :  std_logic;
    signal    regs_ack              :  std_logic;
    signal    regs_err              :  std_logic;
    signal    regs_addr             :  std_logic_vector(REGS_ADDR_WIDTH  -1 downto 0);
    signal    regs_ben              :  std_logic_vector(REGS_DATA_WIDTH/8-1 downto 0);
    signal    regs_wdata            :  std_logic_vector(REGS_DATA_WIDTH  -1 downto 0);
    signal    regs_rdata            :  std_logic_vector(REGS_DATA_WIDTH  -1 downto 0);
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    ctrl_in_c_by_word     :  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
    signal    ctrl_in_w             :  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
    signal    ctrl_in_h             :  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
    signal    ctrl_out_c            :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    ctrl_out_w            :  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
    signal    ctrl_out_h            :  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
    signal    ctrl_k_w              :  std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
    signal    ctrl_k_h              :  std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
    signal    ctrl_pad_size         :  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
    signal    ctrl_use_th           :  std_logic;
    signal    ctrl_req_valid        :  std_logic;
    signal    ctrl_req_ready        :  std_logic;
    signal    ctrl_res_valid        :  std_logic;
    signal    ctrl_res_ready        :  std_logic;
    signal    ctrl_res_status       :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    signal    core_in_c_by_word     :  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
    signal    core_in_w             :  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
    signal    core_in_h             :  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
    signal    core_out_c            :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    core_out_w            :  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
    signal    core_out_h            :  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
    signal    core_k_w              :  std_logic_vector(QCONV_PARAM.K_W_BITS         -1 downto 0);
    signal    core_k_h              :  std_logic_vector(QCONV_PARAM.K_H_BITS         -1 downto 0);
    signal    core_l_pad_size       :  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
    signal    core_r_pad_size       :  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
    signal    core_t_pad_size       :  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
    signal    core_b_pad_size       :  std_logic_vector(QCONV_PARAM.PAD_SIZE_BITS    -1 downto 0);
    signal    core_use_th           :  std_logic;
    signal    core_param_in         :  std_logic;
    signal    core_req_valid        :  std_logic;
    signal    core_req_ready        :  std_logic;
    signal    core_res_valid        :  std_logic;
    signal    core_res_ready        :  std_logic;
    constant  core_res_status       :  std_logic := '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  I_DATA_WIDTH          :  integer := QCONV_PARAM.NBITS_IN_DATA*QCONV_PARAM.NBITS_PER_WORD;
    signal    i_data_addr           :  std_logic_vector(DATA_ADDR_WIDTH              -1 downto 0);
    signal    i_data                :  std_logic_vector(I_DATA_WIDTH                 -1 downto 0);
    signal    i_data_last           :  std_logic;
    signal    i_data_valid          :  std_logic;
    signal    i_data_ready          :  std_logic;
    signal    i_in_c                :  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
    signal    i_in_w                :  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
    signal    i_in_h                :  std_logic_vector(QCONV_PARAM.IN_H_BITS        -1 downto 0);
    signal    i_x_pos               :  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
    signal    i_x_size              :  std_logic_vector(QCONV_PARAM.IN_W_BITS        -1 downto 0);
    signal    i_req_valid           :  std_logic;
    signal    i_req_ready           :  std_logic;
    signal    i_res_valid           :  std_logic;
    signal    i_res_ready           :  std_logic;
    signal    i_res_none            :  std_logic;
    signal    i_res_error           :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  O_DATA_WIDTH          :  integer := 64;
    signal    o_data_addr           :  std_logic_vector(DATA_ADDR_WIDTH              -1 downto 0);
    signal    o_data                :  std_logic_vector(O_DATA_WIDTH                 -1 downto 0);
    constant  o_data_strb           :  std_logic_vector(O_DATA_WIDTH/8               -1 downto 0) := (others => '1');
    signal    o_data_last           :  std_logic;
    signal    o_data_valid          :  std_logic;
    signal    o_data_ready          :  std_logic;
    signal    o_out_c               :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    o_out_w               :  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
    signal    o_out_h               :  std_logic_vector(QCONV_PARAM.OUT_H_BITS       -1 downto 0);
    signal    o_c_pos               :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    o_c_size              :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    o_x_pos               :  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
    signal    o_x_size              :  std_logic_vector(QCONV_PARAM.OUT_W_BITS       -1 downto 0);
    signal    o_use_th              :  std_logic;
    signal    o_req_valid           :  std_logic;
    signal    o_req_ready           :  std_logic;
    signal    o_res_valid           :  std_logic;
    signal    o_res_ready           :  std_logic;
    signal    o_res_none            :  std_logic;
    signal    o_res_error           :  std_logic;
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  K_DATA_WIDTH          :  integer := QCONV_PARAM.NBITS_K_DATA*QCONV_PARAM.NBITS_PER_WORD;
    signal    k_data_addr           :  std_logic_vector(DATA_ADDR_WIDTH              -1 downto 0);
    signal    k_data                :  std_logic_vector(K_DATA_WIDTH                 -1 downto 0);
    signal    k_data_last           :  std_logic;
    signal    k_data_valid          :  std_logic;
    signal    k_data_ready          :  std_logic;
    signal    k_in_c_by_word        :  std_logic_vector(QCONV_PARAM.IN_C_BY_WORD_BITS-1 downto 0);
    signal    k_out_c               :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    k_out_c_pos           :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    k_out_c_size          :  std_logic_vector(QCONV_PARAM.OUT_C_BITS       -1 downto 0);
    signal    k_k3x3                :  std_logic;
    signal    k_req_valid           :  std_logic;
    signal    k_req_ready           :  std_logic;
    signal    k_res_valid           :  std_logic;
    signal    k_res_ready           :  std_logic;
    signal    k_res_none            :  std_logic;
    signal    k_res_error           :  std_logic;
    
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    constant  T_DATA_WIDTH          :  integer := QCONV_PARAM.NBITS_OUT_DATA*QCONV_PARAM.NUM_THRESHOLDS;
    signal    t_data_addr           :  std_logic_vector(DATA_ADDR_WIDTH              -1 downto 0);
    signal    t_data                :  std_logic_vector(T_DATA_WIDTH                 -1 downto 0);
    signal    t_data_last           :  std_logic;
    signal    t_data_valid          :  std_logic;
    signal    t_data_ready          :  std_logic;
    signal    t_out_c               :  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
    signal    t_out_c_pos           :  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
    signal    t_out_c_size          :  std_logic_vector(QCONV_PARAM.OUT_C_BITS-1 downto 0);
    signal    t_req_valid           :  std_logic;
    signal    t_req_ready           :  std_logic;
    signal    t_res_valid           :  std_logic;
    signal    t_res_ready           :  std_logic;
    signal    t_res_none            :  std_logic;
    signal    t_res_error           :  std_logic;
begin
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    RESET <= '1' when (ARESETn = '0') else '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    S_AXI_IF: AXI4_REGISTER_INTERFACE                -- 
        generic map (                                -- 
            AXI4_ADDR_WIDTH => S_AXI_ADDR_WIDTH    , --
            AXI4_DATA_WIDTH => S_AXI_DATA_WIDTH    , --
            AXI4_ID_WIDTH   => S_AXI_ID_WIDTH      , --
            REGS_ADDR_WIDTH => REGS_ADDR_WIDTH     , --
            REGS_DATA_WIDTH => REGS_DATA_WIDTH       --
        )                                            -- 
        port map (                                   -- 
        ---------------------------------------------------------------------------
        -- Clock and Reset Signals.
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            ARID            => S_AXI_ARID          , -- In  :
            ARADDR          => S_AXI_ARADDR        , -- In  :
            ARLEN           => S_AXI_ARLEN         , -- In  :
            ARSIZE          => S_AXI_ARSIZE        , -- In  :
            ARBURST         => S_AXI_ARBURST       , -- In  :
            ARVALID         => S_AXI_ARVALID       , -- In  :
            ARREADY         => S_AXI_ARREADY       , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            RID             => S_AXI_RID           , -- Out :
            RDATA           => S_AXI_RDATA         , -- Out :
            RRESP           => S_AXI_RRESP         , -- Out :
            RLAST           => S_AXI_RLAST         , -- Out :
            RVALID          => S_AXI_RVALID        , -- Out :
            RREADY          => S_AXI_RREADY        , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Write Address Channel Signals.
        ---------------------------------------------------------------------------
            AWID            => S_AXI_AWID          , -- In  :
            AWADDR          => S_AXI_AWADDR        , -- In  :
            AWLEN           => S_AXI_AWLEN         , -- In  :
            AWSIZE          => S_AXI_AWSIZE        , -- In  :
            AWBURST         => S_AXI_AWBURST       , -- In  :
            AWVALID         => S_AXI_AWVALID       , -- In  :
            AWREADY         => S_AXI_AWREADY       , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Write Data Channel Signals.
        ---------------------------------------------------------------------------
            WDATA           => S_AXI_WDATA         , -- In  :
            WSTRB           => S_AXI_WSTRB         , -- In  :
            WLAST           => S_AXI_WLAST         , -- In  :
            WVALID          => S_AXI_WVALID        , -- In  :
            WREADY          => S_AXI_WREADY        , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Write Response Channel Signals.
        ---------------------------------------------------------------------------
            BID             => S_AXI_BID           , -- Out :
            BRESP           => S_AXI_BRESP         , -- Out :
            BVALID          => S_AXI_BVALID        , -- Out :
            BREADY          => S_AXI_BREADY        , -- In  :
        ---------------------------------------------------------------------------
        -- Register Interface.
        ---------------------------------------------------------------------------
            REGS_REQ        => regs_req            , -- Out :
            REGS_WRITE      => regs_write          , -- Out :
            REGS_ACK        => regs_ack            , -- In  :
            REGS_ERR        => regs_err            , -- In  :
            REGS_ADDR       => regs_addr           , -- Out :
            REGS_BEN        => regs_ben            , -- Out :
            REGS_WDATA      => regs_wdata          , -- Out :
            REGS_RDATA      => regs_rdata            -- In  :
        );                                           -- 
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    I_AXI_IF: QCONV_STRIP_IN_DATA_AXI_READER         -- 
        generic map (                                -- 
            QCONV_PARAM     => QCONV_PARAM         , --
            AXI_ADDR_WIDTH  => I_AXI_ADDR_WIDTH    , --
            AXI_DATA_WIDTH  => I_AXI_DATA_WIDTH    , --
            AXI_ID_WIDTH    => I_AXI_ID_WIDTH      , --
            AXI_USER_WIDTH  => I_AXI_USER_WIDTH    , --
            AXI_XFER_SIZE   => I_AXI_XFER_SIZE     , --
            AXI_ID          => I_AXI_ID            , --
            AXI_PROT        => I_AXI_PROT          , --
            AXI_QOS         => I_AXI_QOS           , --
            AXI_REGION      => I_AXI_REGION        , -- 
            AXI_CACHE       => I_AXI_CACHE         , --
            AXI_REQ_QUEUE   => I_AXI_REQ_QUEUE     , --
            REQ_ADDR_WIDTH  => DATA_ADDR_WIDTH       -- 
        )                                            -- 
        port map (                                   -- 
        ---------------------------------------------------------------------------
        -- Clock / Reset Signals.
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            AXI_ARID        => IO_AXI_ARID         , -- Out :
            AXI_ARADDR      => IO_AXI_ARADDR       , -- Out :
            AXI_ARLEN       => IO_AXI_ARLEN        , -- Out :
            AXI_ARSIZE      => IO_AXI_ARSIZE       , -- Out :
            AXI_ARBURST     => IO_AXI_ARBURST      , -- Out :
            AXI_ARLOCK      => IO_AXI_ARLOCK       , -- Out :
            AXI_ARCACHE     => IO_AXI_ARCACHE      , -- Out :
            AXI_ARPROT      => IO_AXI_ARPROT       , -- Out :
            AXI_ARQOS       => IO_AXI_ARQOS        , -- Out :
            AXI_ARREGION    => IO_AXI_ARREGION     , -- Out :
            AXI_ARUSER      => IO_AXI_ARUSER       , -- Out :
            AXI_ARVALID     => IO_AXI_ARVALID      , -- Out :
            AXI_ARREADY     => IO_AXI_ARREADY      , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            AXI_RID         => IO_AXI_RID          , -- In  :
            AXI_RDATA       => IO_AXI_RDATA        , -- In  :
            AXI_RRESP       => IO_AXI_RRESP        , -- In  :
            AXI_RLAST       => IO_AXI_RLAST        , -- In  :
            AXI_RVALID      => IO_AXI_RVALID       , -- In  :
            AXI_RREADY      => IO_AXI_RREADY       , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Stream Master Interface.
        ---------------------------------------------------------------------------
            O_DATA          => i_data              , -- Out :
            O_LAST          => i_data_last         , -- Out :
            O_VALID         => i_data_valid        , -- Out :
            O_READY         => i_data_ready        , -- In  :
        ---------------------------------------------------------------------------
        -- Request / Response Interface.
        ---------------------------------------------------------------------------
            REQ_ADDR        => i_data_addr         , -- In  :
            REQ_IN_C        => i_in_c              , -- In  :
            REQ_IN_W        => i_in_w              , -- In  :
            REQ_IN_H        => i_in_h              , -- In  :
            REQ_X_POS       => i_x_pos             , -- In  :
            REQ_X_SIZE      => i_x_size            , -- In  :
            REQ_VALID       => i_req_valid         , -- In  :
            REQ_READY       => i_req_ready         , -- Out :
            RES_VALID       => i_res_valid         , -- Out :
            RES_NONE        => i_res_none          , -- Out :
            RES_ERROR       => i_res_error         , -- Out :
            RES_READY       => i_res_ready           -- In  :
        );
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    O_AXI_IF: QCONV_STRIP_OUT_DATA_AXI_WRITER        -- 
        generic map (                                -- 
            QCONV_PARAM     => QCONV_PARAM         , --
            AXI_ADDR_WIDTH  => O_AXI_ADDR_WIDTH    , --
            AXI_DATA_WIDTH  => O_AXI_DATA_WIDTH    , --
            AXI_ID_WIDTH    => O_AXI_ID_WIDTH      , --
            AXI_USER_WIDTH  => O_AXI_USER_WIDTH    , --
            AXI_XFER_SIZE   => O_AXI_XFER_SIZE     , --
            AXI_ID          => O_AXI_ID            , --
            AXI_PROT        => O_AXI_PROT          , --
            AXI_QOS         => O_AXI_QOS           , --
            AXI_REGION      => O_AXI_REGION        , --
            AXI_CACHE       => O_AXI_CACHE         , --
            AXI_REQ_QUEUE   => O_AXI_REQ_QUEUE     , --
            I_DATA_WIDTH    => O_DATA_WIDTH        , --
            REQ_ADDR_WIDTH  => DATA_ADDR_WIDTH       --
        )                                            -- 
        port map(                                    -- 
        ---------------------------------------------------------------------------
        -- Clock / Reset Signals.
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Write Address Channel Signals.
        ---------------------------------------------------------------------------
            AXI_AWID        => IO_AXI_AWID         , -- Out :
            AXI_AWADDR      => IO_AXI_AWADDR       , -- Out :
            AXI_AWLEN       => IO_AXI_AWLEN        , -- Out :
            AXI_AWSIZE      => IO_AXI_AWSIZE       , -- Out :
            AXI_AWBURST     => IO_AXI_AWBURST      , -- Out :
            AXI_AWLOCK      => IO_AXI_AWLOCK       , -- Out :
            AXI_AWCACHE     => IO_AXI_AWCACHE      , -- Out :
            AXI_AWPROT      => IO_AXI_AWPROT       , -- Out :
            AXI_AWQOS       => IO_AXI_AWQOS        , -- Out :
            AXI_AWREGION    => IO_AXI_AWREGION     , -- Out :
            AXI_AWUSER      => IO_AXI_AWUSER       , -- Out :
            AXI_AWVALID     => IO_AXI_AWVALID      , -- Out :
            AXI_AWREADY     => IO_AXI_AWREADY      , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Write Data Channel Signals.
        ---------------------------------------------------------------------------
            AXI_WID         => IO_AXI_WID          , -- Out :
            AXI_WDATA       => IO_AXI_WDATA        , -- Out :
            AXI_WSTRB       => IO_AXI_WSTRB        , -- Out :
            AXI_WLAST       => IO_AXI_WLAST        , -- Out :
            AXI_WVALID      => IO_AXI_WVALID       , -- Out :
            AXI_WREADY      => IO_AXI_WREADY       , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Write Response Channel Signals.
        ---------------------------------------------------------------------------
            AXI_BID         => IO_AXI_BID          , -- In  :
            AXI_BRESP       => IO_AXI_BRESP        , -- In  :
            AXI_BVALID      => IO_AXI_BVALID       , -- In  :
            AXI_BREADY      => IO_AXI_BREADY       , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Stream Slave Interface.
        ---------------------------------------------------------------------------
            I_DATA          => o_data              , -- In  :
            I_STRB          => o_data_strb         , -- In  :
            I_LAST          => o_data_last         , -- In  :
            I_VALID         => o_data_valid        , -- In  :
            I_READY         => o_data_ready        , -- Out :
        ---------------------------------------------------------------------------
        -- Request / Response Interface.
        ---------------------------------------------------------------------------
            REQ_ADDR        => o_data_addr         , -- In  :
            REQ_OUT_C       => o_out_c             , -- In  :
            REQ_OUT_W       => o_out_w             , -- In  :
            REQ_OUT_H       => o_out_h             , -- In  :
            REQ_C_POS       => o_c_pos             , -- In  :
            REQ_C_SIZE      => o_c_size            , -- In  :
            REQ_X_POS       => o_x_pos             , -- In  :
            REQ_X_SIZE      => o_x_size            , -- In  :
            REQ_USE_TH      => o_use_th            , -- In  :
            REQ_VALID       => o_req_valid         , -- In  :
            REQ_READY       => o_req_ready         , -- Out :
            RES_VALID       => o_res_valid         , -- Out :
            RES_NONE        => o_res_none          , -- Out :
            RES_ERROR       => o_res_error         , -- Out :
            RES_READY       => o_res_ready           -- In  :
        );
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    K_AXI_IF: QCONV_STRIP_K_DATA_AXI_READER          -- 
        generic map (                                -- 
            QCONV_PARAM     => QCONV_PARAM         , -- 
            AXI_ADDR_WIDTH  => K_AXI_ADDR_WIDTH    , --
            AXI_DATA_WIDTH  => K_AXI_DATA_WIDTH    , --
            AXI_ID_WIDTH    => K_AXI_ID_WIDTH      , --
            AXI_USER_WIDTH  => K_AXI_USER_WIDTH    , --
            AXI_XFER_SIZE   => K_AXI_XFER_SIZE     , --
            AXI_ID          => K_AXI_ID            , --
            AXI_PROT        => K_AXI_PROT          , --
            AXI_QOS         => K_AXI_QOS           , --
            AXI_REGION      => K_AXI_REGION        , --
            AXI_CACHE       => K_AXI_CACHE         , --
            AXI_REQ_QUEUE   => K_AXI_REQ_QUEUE     , --
            REQ_ADDR_WIDTH  => DATA_ADDR_WIDTH       --
        )                                            -- 
        port map (                                   -- 
        ---------------------------------------------------------------------------
        -- Clock / Reset Signals.
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            AXI_ARID        => K_AXI_ARID          , -- Out :
            AXI_ARADDR      => K_AXI_ARADDR        , -- Out :
            AXI_ARLEN       => K_AXI_ARLEN         , -- Out :
            AXI_ARSIZE      => K_AXI_ARSIZE        , -- Out :
            AXI_ARBURST     => K_AXI_ARBURST       , -- Out :
            AXI_ARLOCK      => K_AXI_ARLOCK        , -- Out :
            AXI_ARCACHE     => K_AXI_ARCACHE       , -- Out :
            AXI_ARPROT      => K_AXI_ARPROT        , -- Out :
            AXI_ARQOS       => K_AXI_ARQOS         , -- Out :
            AXI_ARREGION    => K_AXI_ARREGION      , -- Out :
            AXI_ARUSER      => K_AXI_ARUSER        , -- Out :
            AXI_ARVALID     => K_AXI_ARVALID       , -- Out :
            AXI_ARREADY     => K_AXI_ARREADY       , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            AXI_RID         => K_AXI_RID           , -- In  :
            AXI_RDATA       => K_AXI_RDATA         , -- In  :
            AXI_RRESP       => K_AXI_RRESP         , -- In  :
            AXI_RLAST       => K_AXI_RLAST         , -- In  :
            AXI_RVALID      => K_AXI_RVALID        , -- In  :
            AXI_RREADY      => K_AXI_RREADY        , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Stream Master Interface.
        ---------------------------------------------------------------------------
            O_DATA          => k_data              , -- Out :
            O_LAST          => k_data_last         , -- Out :
            O_VALID         => k_data_valid        , -- Out :
            O_READY         => k_data_ready        , -- In  :
        ---------------------------------------------------------------------------
        -- Request / Response Interface.
        ---------------------------------------------------------------------------
            REQ_ADDR        => k_data_addr         , -- In  :
            REQ_IN_C        => k_in_c_by_word      , -- In  :
            REQ_OUT_C       => k_out_c             , -- In  :
            REQ_OUT_C_POS   => k_out_c_pos         , -- In  :
            REQ_OUT_C_SIZE  => k_out_c_size        , -- In  :
            REQ_K3x3        => k_k3x3              , -- In  :
            REQ_VALID       => k_req_valid         , -- In  :
            REQ_READY       => k_req_ready         , -- Out :
            RES_VALID       => k_res_valid         , -- Out :
            RES_NONE        => k_res_none          , -- Out :
            RES_ERROR       => k_res_error         , -- Out :
            RES_READY       => k_res_ready           -- In  :
        );
    K_AXI_AWID     <= (others => '0');
    K_AXI_AWADDR   <= (others => '0');
    K_AXI_AWLEN    <= (others => '0');
    K_AXI_AWSIZE   <= (others => '0');
    K_AXI_AWBURST  <= (others => '0');
    K_AXI_AWLOCK   <= (others => '0');
    K_AXI_AWCACHE  <= (others => '0');
    K_AXI_AWPROT   <= (others => '0');
    K_AXI_AWQOS    <= (others => '0');
    K_AXI_AWREGION <= (others => '0');
    K_AXI_AWUSER   <= (others => '0');
    K_AXI_AWVALID  <= '0';
    K_AXI_WID      <= (others => '0');
    K_AXI_WDATA    <= (others => '0');
    K_AXI_WSTRB    <= (others => '0');
    K_AXI_WLAST    <= '0';
    K_AXI_WVALID   <= '0';
    K_AXI_BREADY   <= '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    T_AXI_IF: QCONV_STRIP_TH_DATA_AXI_READER         -- 
        generic map (                                -- 
            QCONV_PARAM     => QCONV_PARAM         , -- 
            AXI_ADDR_WIDTH  => T_AXI_ADDR_WIDTH    , --
            AXI_DATA_WIDTH  => T_AXI_DATA_WIDTH    , --
            AXI_ID_WIDTH    => T_AXI_ID_WIDTH      , --
            AXI_USER_WIDTH  => T_AXI_USER_WIDTH    , --
            AXI_XFER_SIZE   => T_AXI_XFER_SIZE     , --
            AXI_ID          => T_AXI_ID            , --
            AXI_PROT        => T_AXI_PROT          , --
            AXI_QOS         => T_AXI_QOS           , --
            AXI_REGION      => T_AXI_REGION        , --
            AXI_CACHE       => T_AXI_CACHE         , --
            AXI_REQ_QUEUE   => T_AXI_REQ_QUEUE     , --
            REQ_ADDR_WIDTH  => DATA_ADDR_WIDTH       --
        )                                            -- 
        port map (                                   -- 
        ---------------------------------------------------------------------------
        -- Clock / Reset Signals.
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Address Channel Signals.
        ---------------------------------------------------------------------------
            AXI_ARID        => T_AXI_ARID          , -- Out :
            AXI_ARADDR      => T_AXI_ARADDR        , -- Out :
            AXI_ARLEN       => T_AXI_ARLEN         , -- Out :
            AXI_ARSIZE      => T_AXI_ARSIZE        , -- Out :
            AXI_ARBURST     => T_AXI_ARBURST       , -- Out :
            AXI_ARLOCK      => T_AXI_ARLOCK        , -- Out :
            AXI_ARCACHE     => T_AXI_ARCACHE       , -- Out :
            AXI_ARPROT      => T_AXI_ARPROT        , -- Out :
            AXI_ARQOS       => T_AXI_ARQOS         , -- Out :
            AXI_ARREGION    => T_AXI_ARREGION      , -- Out :
            AXI_ARUSER      => T_AXI_ARUSER        , -- Out :
            AXI_ARVALID     => T_AXI_ARVALID       , -- Out :
            AXI_ARREADY     => T_AXI_ARREADY       , -- In  :
        ---------------------------------------------------------------------------
        -- AXI4 Read Data Channel Signals.
        ---------------------------------------------------------------------------
            AXI_RID         => T_AXI_RID           , -- In  :
            AXI_RDATA       => T_AXI_RDATA         , -- In  :
            AXI_RRESP       => T_AXI_RRESP         , -- In  :
            AXI_RLAST       => T_AXI_RLAST         , -- In  :
            AXI_RVALID      => T_AXI_RVALID        , -- In  :
            AXI_RREADY      => T_AXI_RREADY        , -- Out :
        ---------------------------------------------------------------------------
        -- AXI4 Stream Master Interface.
        ---------------------------------------------------------------------------
            O_DATA          => t_data              , -- Out :
            O_LAST          => t_data_last         , -- Out :
            O_VALID         => t_data_valid        , -- Out :
            O_READY         => t_data_ready        , -- In  :
        ---------------------------------------------------------------------------
        -- Request / Response Interface.
        ---------------------------------------------------------------------------
            REQ_ADDR        => t_data_addr         , -- In  :
            REQ_OUT_C       => t_out_c             , -- In  :
            REQ_OUT_C_POS   => t_out_c_pos         , -- In  :
            REQ_OUT_C_SIZE  => t_out_c_size        , -- In  :
            REQ_VALID       => t_req_valid         , -- In  :
            REQ_READY       => t_req_ready         , -- Out :
            RES_VALID       => t_res_valid         , -- Out :
            RES_NONE        => t_res_none          , -- Out :
            RES_ERROR       => t_res_error         , -- Out :
            RES_READY       => t_res_ready           -- In  :
        );
    T_AXI_AWID     <= (others => '0');
    T_AXI_AWADDR   <= (others => '0');
    T_AXI_AWLEN    <= (others => '0');
    T_AXI_AWSIZE   <= (others => '0');
    T_AXI_AWBURST  <= (others => '0');
    T_AXI_AWLOCK   <= (others => '0');
    T_AXI_AWCACHE  <= (others => '0');
    T_AXI_AWPROT   <= (others => '0');
    T_AXI_AWQOS    <= (others => '0');
    T_AXI_AWREGION <= (others => '0');
    T_AXI_AWUSER   <= (others => '0');
    T_AXI_AWVALID  <= '0';
    T_AXI_WID      <= (others => '0');
    T_AXI_WDATA    <= (others => '0');
    T_AXI_WSTRB    <= (others => '0');
    T_AXI_WLAST    <= '0';
    T_AXI_WVALID   <= '0';
    T_AXI_BREADY   <= '0';
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    REGS: QCONV_STRIP_REGISTERS                      -- 
        generic map (                                -- 
            ID              => ID                  , -- 
            QCONV_PARAM     => QCONV_PARAM         , -- 
            DATA_ADDR_WIDTH => DATA_ADDR_WIDTH     , -- 
            REGS_ADDR_WIDTH => REGS_ADDR_WIDTH     , -- 
            REGS_DATA_WIDTH => REGS_DATA_WIDTH       -- 
        )                                            -- 
        port map (                                   -- 
        ---------------------------------------------------------------------------
        -- クロック&リセット信号
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- Register Access Interface
        ---------------------------------------------------------------------------
            REGS_REQ        => regs_req            , -- In  :
            REGS_WRITE      => regs_write          , -- In  :
            REGS_ADDR       => regs_addr           , -- In  :
            REGS_BEN        => regs_ben            , -- In  :
            REGS_WDATA      => regs_wdata          , -- In  :
            REGS_RDATA      => regs_rdata          , -- Out :
            REGS_ACK        => regs_ack            , -- Out :
            REGS_ERR        => regs_err            , -- Out :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Registers
        ---------------------------------------------------------------------------
            I_DATA_ADDR     => i_data_addr         , -- Out :
            O_DATA_ADDR     => o_data_addr         , -- Out :
            K_DATA_ADDR     => k_data_addr         , -- Out :
            T_DATA_ADDR     => t_data_addr         , -- Out :
            I_WIDTH         => ctrl_in_w           , -- Out :
            I_HEIGHT        => ctrl_in_h           , -- Out :
            I_CHANNELS      => ctrl_in_c_by_word   , -- Out :
            O_WIDTH         => ctrl_out_w          , -- Out :
            O_HEIGHT        => ctrl_out_h          , -- Out :
            O_CHANNELS      => ctrl_out_c          , -- Out :
            K_WIDTH         => ctrl_k_w            , -- Out :
            K_HEIGHT        => ctrl_k_h            , -- Out :
            PAD_SIZE        => ctrl_pad_size       , -- Out :
            USE_TH          => ctrl_use_th         , -- Out :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Request/Response Interface
        ---------------------------------------------------------------------------
            REQ_VALID       => ctrl_req_valid      , -- Out :
            REQ_READY       => ctrl_req_ready      , -- In  :
            RES_VALID       => ctrl_res_valid      , -- In  :
            RES_READY       => ctrl_res_ready      , -- Out :
            RES_STATUS      => ctrl_res_status     , -- In  :
            REQ_RESET       => open                , -- Out :
            REQ_STOP        => open                , -- Out :
            REQ_PAUSE       => open                , -- Out :
        ---------------------------------------------------------------------------
        -- Interrupt Request 
        ---------------------------------------------------------------------------
            IRQ             => IRQ                   -- Out :
        );
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    CTRL: QCONV_STRIP_CONTROLLER                     -- 
        generic map (                                -- 
            QCONV_PARAM     => QCONV_PARAM         , -- 
            IN_BUF_SIZE     => IN_BUF_SIZE         , --
            K_BUF_SIZE      => K_BUF_SIZE          , --
            IN_C_UNROLL     => IN_C_UNROLL           --
        )                                            -- 
        port map (                                   -- 
        ---------------------------------------------------------------------------
        -- クロック&リセット信号
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Register Interface
        ---------------------------------------------------------------------------
            IN_C_BY_WORD    => ctrl_in_c_by_word   , -- In  :
            IN_W            => ctrl_in_w           , -- In  :
            IN_H            => ctrl_in_h           , -- In  :
            OUT_C           => ctrl_out_c          , -- In  :
            OUT_W           => ctrl_out_w          , -- In  :
            OUT_H           => ctrl_out_h          , -- In  :
            K_W             => ctrl_k_w            , -- In  :
            K_H             => ctrl_k_h            , -- In  :
            PAD_SIZE        => ctrl_pad_size       , -- In  :
            USE_TH          => ctrl_use_th         , -- In  :
            REQ_VALID       => ctrl_req_valid      , -- In  :
            REQ_READY       => ctrl_req_ready      , -- Out :
            RES_VALID       => ctrl_res_valid      , -- Out :
            RES_READY       => ctrl_res_ready      , -- In  :
            RES_STATUS      => ctrl_res_status     , -- Out :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Core Module Interface
        ---------------------------------------------------------------------------
            CORE_IN_C       => core_in_c_by_word   , -- Out :
            CORE_IN_W       => core_in_w           , -- Out :
            CORE_IN_H       => core_in_h           , -- Out :
            CORE_OUT_C      => core_out_c          , -- Out :
            CORE_OUT_W      => core_out_w          , -- Out :
            CORE_OUT_H      => core_out_h          , -- Out :
            CORE_K_W        => core_k_w            , -- Out :
            CORE_K_H        => core_k_h            , -- Out :
            CORE_L_PAD_SIZE => core_l_pad_size     , -- Out :
            CORE_R_PAD_SIZE => core_r_pad_size     , -- Out :
            CORE_T_PAD_SIZE => core_t_pad_size     , -- Out :
            CORE_B_PAD_SIZE => core_b_pad_size     , -- Out :
            CORE_USE_TH     => core_use_th         , -- Out :
            CORE_PARAM_IN   => core_param_in       , -- Out :
            CORE_REQ_VALID  => core_req_valid      , -- Out :
            CORE_REQ_READY  => core_req_ready      , -- In  :
            CORE_RES_VALID  => core_res_valid      , -- In  :
            CORE_RES_READY  => core_res_ready      , -- Out :
            CORE_RES_STATUS => core_res_status     , -- In  :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) In Data AXI Reader Module Interface
        ---------------------------------------------------------------------------
            I_IN_C          => i_in_c              , -- Out :
            I_IN_W          => i_in_w              , -- Out :
            I_IN_H          => i_in_h              , -- Out :
            I_X_POS         => i_x_pos             , -- Out :
            I_X_SIZE        => i_x_size            , -- Out :
            I_REQ_VALID     => i_req_valid         , -- Out :
            I_REQ_READY     => i_req_ready         , -- In  :
            I_RES_VALID     => i_res_valid         , -- In  :
            I_RES_READY     => i_res_ready         , -- Out :
            I_RES_NONE      => i_res_none          , -- In  :
            I_RES_ERROR     => i_res_error         , -- In  :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Kernel Weight Data AXI Reader Module Interface
        ---------------------------------------------------------------------------
            K_IN_C          => k_in_c_by_word      , -- Out :
            K_OUT_C         => k_out_c             , -- Out :
            K_OUT_C_POS     => k_out_c_pos         , -- Out :
            K_OUT_C_SIZE    => k_out_c_size        , -- Out :
            K_REQ_K3x3      => k_k3x3              , -- Out :
            K_REQ_VALID     => k_req_valid         , -- Out :
            K_REQ_READY     => k_req_ready         , -- In  :
            K_RES_VALID     => k_res_valid         , -- In  :
            K_RES_READY     => k_res_ready         , -- Out :
            K_RES_NONE      => k_res_none          , -- In  :
            K_RES_ERROR     => k_res_error         , -- In  :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Thresholds Data AXI Reader Module Interface
        ---------------------------------------------------------------------------
            T_OUT_C         => t_out_c             , -- Out :
            T_OUT_C_POS     => t_out_c_pos         , -- Out :
            T_OUT_C_SIZE    => t_out_c_size        , -- Out :
            T_REQ_VALID     => t_req_valid         , -- Out :
            T_REQ_READY     => t_req_ready         , -- In  :
            T_RES_VALID     => t_res_valid         , -- In  :
            T_RES_READY     => t_res_ready         , -- Out :
            T_RES_NONE      => t_res_none          , -- In  :
            T_RES_ERROR     => t_res_error         , -- In  :
        ---------------------------------------------------------------------------
        -- Quantized Convolution (strip) Out Data AXI Writer Module Interface
        ---------------------------------------------------------------------------
            O_OUT_C         => o_out_c             , -- Out :
            O_OUT_W         => o_out_w             , -- Out :
            O_OUT_H         => o_out_h             , -- Out :
            O_C_POS         => o_c_pos             , -- Out :
            O_C_SIZE        => o_c_size            , -- Out :
            O_X_POS         => o_x_pos             , -- Out :
            O_X_SIZE        => o_x_size            , -- Out :
            O_USE_TH        => o_use_th            , -- Out :
            O_REQ_VALID     => o_req_valid         , -- Out :
            O_REQ_READY     => o_req_ready         , -- In  :
            O_RES_VALID     => o_res_valid         , -- In  :
            O_RES_READY     => o_res_ready         , -- Out :
            O_RES_NONE      => o_res_none          , -- In  :
            O_RES_ERROR     => o_res_error           -- In  :
        );
    -------------------------------------------------------------------------------
    --
    -------------------------------------------------------------------------------
    CORE: QCONV_STRIP_CORE
        generic map (                                -- 
            QCONV_PARAM     => QCONV_PARAM         , -- 
            IN_BUF_SIZE     => IN_BUF_SIZE         , -- 
            K_BUF_SIZE      => K_BUF_SIZE          , -- 
            TH_BUF_SIZE     => TH_BUF_SIZE         , -- 
            IN_C_UNROLL     => IN_C_UNROLL         , -- 
            OUT_C_UNROLL    => OUT_C_UNROLL        , -- 
            OUT_DATA_BITS   => O_DATA_WIDTH          -- 
        )
        port map (
        ---------------------------------------------------------------------------
        -- クロック&リセット信号
        ---------------------------------------------------------------------------
            CLK             => ACLK                , -- In  :
            RST             => RESET               , -- In  :
            CLR             => CLEAR               , -- In  :
        ---------------------------------------------------------------------------
        -- 
        ---------------------------------------------------------------------------
            IN_C_BY_WORD    => core_in_c_by_word   , -- In  :
            IN_W            => core_in_w           , -- In  :
            IN_H            => core_in_h           , -- In  :
            OUT_C           => core_out_c          , -- In  :
            OUT_W           => core_out_w          , -- In  :
            OUT_H           => core_out_h          , -- In  :
            K_W             => core_k_w            , -- In  :
            K_H             => core_k_h            , -- In  :
            LEFT_PAD_SIZE   => core_l_pad_size     , -- In  :
            RIGHT_PAD_SIZE  => core_r_pad_size     , -- In  :
            TOP_PAD_SIZE    => core_t_pad_size     , -- In  :
            BOTTOM_PAD_SIZE => core_b_pad_size     , -- In  :
            USE_TH          => core_use_th         , -- In  :
            PARAM_IN        => core_param_in       , -- In  :
            REQ_VALID       => core_req_valid      , -- In  :
            REQ_READY       => core_req_ready      , -- Out :
            RES_VALID       => core_res_valid      , -- Out :
            RES_READY       => core_res_ready      , -- In  :
        ---------------------------------------------------------------------------
        -- データ入力 I/F
        ---------------------------------------------------------------------------
            IN_DATA         => i_data              , -- In  :
            IN_VALID        => i_data_valid        , -- In  :
            IN_READY        => i_data_ready        , -- Out :
        ---------------------------------------------------------------------------
        -- カーネル係数入力 I/F
        ---------------------------------------------------------------------------
            K_DATA          => k_data              , -- In  :
            K_VALID         => k_data_valid        , -- In  :
            K_READY         => k_data_ready        , -- Out :
        ---------------------------------------------------------------------------
        -- スレッシュホールド係数入力 I/F
        ---------------------------------------------------------------------------
            TH_DATA         => t_data              , -- In  :
            TH_VALID        => t_data_valid        , -- In  :
            TH_READY        => t_data_ready        , -- Out :
        ---------------------------------------------------------------------------
        -- データ出力 I/F
        ---------------------------------------------------------------------------
            OUT_DATA        => o_data              , -- Out :
            OUT_LAST        => o_data_last         , -- Out :
            OUT_VALID       => o_data_valid        , -- Out :
            OUT_READY       => o_data_ready          -- In  :
    );
end RTL;

