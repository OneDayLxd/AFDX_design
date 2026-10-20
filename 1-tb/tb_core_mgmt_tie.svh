// afdx_es_core 管理扩展端口的空连接（数据面测试不使用运行时配置与会话元数据）。
        .tx_msg_ep(34'd0), .rx_msg_src_ip(), .rx_msg_sport(), .rx_msg_dport(), .rx_msg_ts(),
        .ts_us(32'd0), .tx_sof_tag_a(), .tx_sof_tag_b(), .mono_us(),
        .cfg_m_rd_idx({TX_VLW_TIE{1'b0}}), .cfg_m_rd_shadow(1'b0), .cfg_m_bag_us(), .cfg_m_lmax(),
        .cfg_m_net_en(), .cfg_m_vlid(), .cfg_m_rom_lmax(), .cfg_m_wr_en(1'b0),
        .cfg_m_wr_idx({TX_VLW_TIE{1'b0}}), .cfg_m_wr_bag_us(20'd0), .cfg_m_wr_lmax(11'd0),
        .cfg_m_wr_net_en(2'd0), .cfg_cmd_copy(1'b0), .cfg_cmd_verify(1'b0), .cfg_cmd_apply(1'b0),
        .cfg_cmd_rollback(1'b0), .cfg_busy(), .cfg_active_bank(), .cfg_prev_valid(), .cfg_done(),
        .cfg_verify_ok(), .cfg_verify_code(), .cfg_verify_idx(), .cfg_budget(),
