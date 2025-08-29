function mouse_log_viewer()
    % マイクロマウスのログデータをリアルタイムでグラフ表示するMATLABスクリプト
    % log_print関数から出力されるCSVデータを解析してグラフ化
    
    % シリアルポートの設定
    port = input('シリアルポート名を入力してください (例: COM3): ', 's');
    baudrate = 115200; % ESP32の標準ボーレート
    
    try
        % シリアルポート接続
        s = serialport(port, baudrate);
        configureTerminator(s, "LF"); % 改行文字で区切り
        
        fprintf('シリアルポート %s に接続しました。\n', port);
        fprintf('ログデータの受信を開始します...\n');
        fprintf('終了するには Ctrl+C を押してください。\n\n');
        
        % データ保存用変数の初期化
        data_buffer = [];
        max_samples = 10000; % 最大サンプル数
        
        % グラフウィンドウの設定
        figure('Name', 'マイクロマウスログビューア', 'Position', [100, 100, 1600, 1200]);
        
        % サブプロット作成
        subplot(4, 3, 1);
        h1 = plot(0, 0, 'b-', 'LineWidth', 1.5);
        title('壁センサ値');
        xlabel('サンプル数');
        ylabel('センサ値');
        legend({'前左', '左', '右', '前右'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 2);
        h2 = plot(0, 0, 'r-', 0, 0, 'g-', 'LineWidth', 1.5);
        title('速度制御');
        xlabel('サンプル数');
        ylabel('速度 [m/s]');
        legend({'現在速度', '目標速度'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 3);
        h3 = plot(0, 0, 'r-', 0, 0, 'g-', 'LineWidth', 1.5);
        title('角速度制御');
        xlabel('サンプル数');
        ylabel('角速度 [rad/s]');
        legend({'現在角速度', '目標角速度'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 4);
        h4 = plot(0, 0, 'b-', 'LineWidth', 1.5);
        title('走行距離');
        xlabel('サンプル数');
        ylabel('距離 [m]');
        legend({'累積距離', '現在距離', '目標距離'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 5);
        h5 = plot(0, 0, 'r-', 0, 0, 'g-', 'LineWidth', 1.5);
        title('角度');
        xlabel('サンプル数');
        ylabel('角度 [rad]');
        legend({'現在角度'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 6);
        h6 = plot(0, 0, 'r-', 0, 0, 'g-', 'LineWidth', 1.5);
        title('加速度');
        xlabel('サンプル数');
        ylabel('加速度 [m/s²]');
        legend({'並進加速度', '角加速度'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 7);
        h7 = plot(0, 0, 'r-', 0, 0, 'g-', 0, 0, 'b-', 'LineWidth', 1.5);
        title('制御エラー');
        xlabel('サンプル数');
        ylabel('エラー値');
        legend({'速度エラー', '速度積分', '速度微分'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 8);
        h8 = plot(0, 0, 'r-', 0, 0, 'g-', 'LineWidth', 1.5);
        title('モータDuty比');
        xlabel('サンプル数');
        ylabel('Duty比');
        legend({'左モータ', '右モータ'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 9);
        h9 = plot(0, 0, 'r-', 0, 0, 'g-', 'LineWidth', 1.5);
        title('エンコーダ値');
        xlabel('サンプル数');
        ylabel('エンコーダ値');
        legend({'左エンコーダ', '右エンコーダ'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 10);
        h10 = plot(0, 0, 'k-', 'LineWidth', 2);
        title('電源電圧');
        xlabel('サンプル数');
        ylabel('電圧 [V]');
        legend({'バッテリ電圧'}, 'Location', 'best');
        grid on;
        ylim([3.0 4.5]); % リチウムイオン電池の電圧範囲
        
        subplot(4, 3, 11);
        h11 = plot(0, 0, 'b-', 'LineWidth', 1.5);
        title('角速度エラー');
        xlabel('サンプル数');
        ylabel('エラー値');
        legend({'角速度エラー', '角速度積分', '角速度微分'}, 'Location', 'best');
        grid on;
        
        subplot(4, 3, 12);
        h12 = plot(0, 0, 'm-', 'LineWidth', 1.5);
        title('処理時間');
        xlabel('サンプル数');
        ylabel('時間 [μs]');
        legend({'制御周期'}, 'Location', 'best');
        grid on;
        
        % データ受信と描画のメインループ
        sample_count = 0;
        
        while true
            try
                % シリアルデータ読み取り
                if s.NumBytesAvailable > 0
                    line = readline(s);
                    line = strip(line); % 前後の空白文字を削除
                    
                    % CSVデータを解析
                    if ~isempty(line) && ~startsWith(line, 'E(') % エラーメッセージを除外
                        data = parse_csv_line(line);
                        
                        if ~isempty(data) && length(data) >= 26
                            sample_count = sample_count + 1;
                            
                            % データをバッファに追加
                            data_buffer = [data_buffer; data];
                            
                            % バッファサイズ制限
                            if size(data_buffer, 1) > max_samples
                                data_buffer = data_buffer(end-max_samples+1:end, :);
                            end
                            
                            % グラフ更新（100サンプルごと）
                            if mod(sample_count, 100) == 0
                                update_plots(data_buffer, h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, h11, h12);
                                drawnow;
                            end
                            
                            % リアルタイム表示（最新データ）
                            if mod(sample_count, 10) == 0
                                display_latest_data(data);
                            end
                        end
                    end
                end
                
                pause(0.001); % CPU負荷軽減
                
            catch ME
                if strcmp(ME.identifier, 'MATLAB:interruption')
                    fprintf('\n\n受信を停止しました（ユーザー中断）。\n');
                    break; % Ctrl+Cで中断
                else
                    fprintf('エラー: %s\n', ME.message);
                end
            end
        end
        
    catch ME
        fprintf('シリアルポート接続エラー: %s\n', ME.message);
        fprintf('利用可能なポートを確認してください。\n');
        return;
    end
    
    % クリーンアップ
    try
        clear s;
        fprintf('シリアルポートを正常に切断しました。\n');
        fprintf('受信完了: 総サンプル数 %d個\n', sample_count);
    catch
        % 何もしない
    end
    
    % データ保存オプション
    if ~isempty(data_buffer)
        save_data = questdlg('受信したデータを保存しますか？', '保存確認', 'はい', 'いいえ', 'いいえ');
        if strcmp(save_data, 'はい')
            save_log_data(data_buffer);
        end
    end
end

function data = parse_csv_line(line)
    % CSVデータを数値配列に変換
    try
        data_str = split(line, ',');
        data = zeros(1, length(data_str));
        
        for i = 1:length(data_str)
            data(i) = str2double(data_str{i});
        end
        
        % NaNがある場合は空配列を返す
        if any(isnan(data))
            data = [];
        end
    catch
        data = [];
    end
end

function update_plots(data_buffer, h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, h11, h12)
    % グラフを更新
    if isempty(data_buffer)
        return;
    end
    
    sample_indices = 1:size(data_buffer, 1);
    
    % データの変換（スケーリング）
    wall_fl = data_buffer(:, 1);
    wall_l = data_buffer(:, 2);
    wall_r = data_buffer(:, 3);
    wall_fr = data_buffer(:, 4);
    battery = data_buffer(:, 5) / 1000; % mV to V
    vel_current = data_buffer(:, 6) / 1000; % mm/s to m/s
    vel_target = data_buffer(:, 7) / 1000;
    sum_len = data_buffer(:, 8) / 1000; % mm to m
    ang_vel_current = data_buffer(:, 9) / 1000; % mrad/s to rad/s
    ang_vel_target = data_buffer(:, 10) / 1000;
    rad_current = data_buffer(:, 11) / 1000; % mrad to rad
    accel_target = data_buffer(:, 12) / 1000; % mm/s² to m/s²
    ang_accel_target = data_buffer(:, 13) / 1000; % mrad/s² to rad/s²
    vel_error = data_buffer(:, 14) / 1000;
    vel_i_error = data_buffer(:, 15) / 1000;
    vel_d_error = data_buffer(:, 16) / 1000;
    ang_error = data_buffer(:, 17) / 1000;
    ang_i_error = data_buffer(:, 18) / 1000;
    ang_d_error = data_buffer(:, 19) / 1000;
    duty_l = data_buffer(:, 20) / 1000;
    duty_r = data_buffer(:, 21) / 1000;
    enc_l = data_buffer(:, 22);
    enc_r = data_buffer(:, 23);
    len_current = data_buffer(:, 24) / 1000;
    len_target = data_buffer(:, 25) / 1000;
    
    % 処理時間とthinking_flagがある場合
    if size(data_buffer, 2) >= 26
        delta_time = data_buffer(:, 26);
    else
        delta_time = zeros(size(sample_indices));
    end
    
    % 壁センサ値
    subplot(4, 3, 1);
    plot(sample_indices, wall_fl, 'b-', sample_indices, wall_l, 'g-', ...
         sample_indices, wall_r, 'r-', sample_indices, wall_fr, 'm-', 'LineWidth', 1.5);
    title('壁センサ値');
    xlabel('サンプル数');
    ylabel('センサ値');
    legend({'前左', '左', '右', '前右'}, 'Location', 'best');
    grid on;
    
    % 速度制御
    subplot(4, 3, 2);
    plot(sample_indices, vel_current, 'r-', sample_indices, vel_target, 'g-', 'LineWidth', 1.5);
    title('速度制御');
    xlabel('サンプル数');
    ylabel('速度 [m/s]');
    legend({'現在速度', '目標速度'}, 'Location', 'best');
    grid on;
    
    % 角速度制御
    subplot(4, 3, 3);
    plot(sample_indices, ang_vel_current, 'r-', sample_indices, ang_vel_target, 'g-', 'LineWidth', 1.5);
    title('角速度制御');
    xlabel('サンプル数');
    ylabel('角速度 [rad/s]');
    legend({'現在角速度', '目標角速度'}, 'Location', 'best');
    grid on;
    
    % 走行距離
    subplot(4, 3, 4);
    plot(sample_indices, sum_len, 'b-', sample_indices, len_current, 'r-', ...
         sample_indices, len_target, 'g-', 'LineWidth', 1.5);
    title('走行距離');
    xlabel('サンプル数');
    ylabel('距離 [m]');
    legend({'累積距離', '現在距離', '目標距離'}, 'Location', 'best');
    grid on;
    
    % 角度
    subplot(4, 3, 5);
    plot(sample_indices, rad_current, 'r-', 'LineWidth', 1.5);
    title('角度');
    xlabel('サンプル数');
    ylabel('角度 [rad]');
    legend({'現在角度'}, 'Location', 'best');
    grid on;
    
    % 加速度
    subplot(4, 3, 6);
    plot(sample_indices, accel_target, 'r-', sample_indices, ang_accel_target, 'g-', 'LineWidth', 1.5);
    title('加速度');
    xlabel('サンプル数');
    ylabel('加速度 [m/s², rad/s²]');
    legend({'並進加速度', '角加速度'}, 'Location', 'best');
    grid on;
    
    % 速度制御エラー
    subplot(4, 3, 7);
    plot(sample_indices, vel_error, 'r-', sample_indices, vel_i_error, 'g-', ...
         sample_indices, vel_d_error, 'b-', 'LineWidth', 1.5);
    title('速度制御エラー');
    xlabel('サンプル数');
    ylabel('エラー値');
    legend({'速度エラー', '速度積分', '速度微分'}, 'Location', 'best');
    grid on;
    
    % モータDuty比
    subplot(4, 3, 8);
    plot(sample_indices, duty_l, 'r-', sample_indices, duty_r, 'g-', 'LineWidth', 1.5);
    title('モータDuty比');
    xlabel('サンプル数');
    ylabel('Duty比');
    legend({'左モータ', '右モータ'}, 'Location', 'best');
    grid on;
    
    % エンコーダ値
    subplot(4, 3, 9);
    plot(sample_indices, enc_l, 'r-', sample_indices, enc_r, 'g-', 'LineWidth', 1.5);
    title('エンコーダ値');
    xlabel('サンプル数');
    ylabel('エンコーダ値');
    legend({'左エンコーダ', '右エンコーダ'}, 'Location', 'best');
    grid on;
    
    % 電源電圧
    subplot(4, 3, 10);
    plot(sample_indices, battery, 'k-', 'LineWidth', 2);
    title('電源電圧');
    xlabel('サンプル数');
    ylabel('電圧 [V]');
    legend({'バッテリ電圧'}, 'Location', 'best');
    grid on;
    ylim([3.0 4.5]); % リチウムイオン電池の電圧範囲
    
    % 警告表示（電圧低下時）
    if ~isempty(battery) && min(battery) < 3.3
        text(length(sample_indices)*0.5, 3.35, '低電圧警告!', ...
             'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center');
    end
    
    % 角速度制御エラー
    subplot(4, 3, 11);
    plot(sample_indices, ang_error, 'r-', sample_indices, ang_i_error, 'g-', ...
         sample_indices, ang_d_error, 'b-', 'LineWidth', 1.5);
    title('角速度制御エラー');
    xlabel('サンプル数');
    ylabel('エラー値');
    legend({'角速度エラー', '角速度積分', '角速度微分'}, 'Location', 'best');
    grid on;
    
    % 処理時間
    subplot(4, 3, 12);
    plot(sample_indices, delta_time, 'm-', 'LineWidth', 1.5);
    title('制御処理時間');
    xlabel('サンプル数');
    ylabel('時間 [μs]');
    legend({'制御周期'}, 'Location', 'best');
    grid on;
    
    % 処理時間オーバー警告（1ms = 1000μs を超えた場合）
    if ~isempty(delta_time) && max(delta_time) > 1000
        text(length(sample_indices)*0.5, max(delta_time)*0.9, '処理時間オーバー!', ...
             'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center');
    end
    subplot(3, 3, 9);
    plot(sample_indices, enc_l, 'r-', sample_indices, enc_r, 'g-', 'LineWidth', 1.5);
    title('エンコーダ値');
    xlabel('サンプル数');
    ylabel('エンコーダ値');
    legend({'左エンコーダ', '右エンコーダ'}, 'Location', 'best');
    grid on;
end

function display_latest_data(data)
    % 最新データをコマンドウィンドウに表示
    if length(data) >= 26
        fprintf('サンプル: 壁センサ[%4d,%4d,%4d,%4d] 速度[%.3f] 角速度[%.3f] バッテリ[%.2fV]\n', ...
            data(1), data(2), data(3), data(4), data(6)/1000, data(9)/1000, data(5)/1000);
    end
end

function save_log_data(data_buffer)
    % データをMATファイルとCSVファイルに保存
    try
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        
        % MATファイル保存
        mat_filename = sprintf('mouse_log_%s.mat', timestamp);
        save(mat_filename, 'data_buffer');
        
        % CSVファイル保存
        csv_filename = sprintf('mouse_log_%s.csv', timestamp);
        
        % ヘッダー作成
        header = {'wall_fl', 'wall_l', 'wall_r', 'wall_fr', 'battery_mV', ...
                  'vel_current_mm_s', 'vel_target_mm_s', 'sum_len_mm', ...
                  'ang_vel_current_mrad_s', 'ang_vel_target_mrad_s', 'rad_current_mrad', ...
                  'accel_target_mm_s2', 'ang_accel_target_mrad_s2', ...
                  'vel_error_mm_s', 'vel_i_error_mm_s', 'vel_d_error_mm_s', ...
                  'ang_error_mrad_s', 'ang_i_error_mrad_s', 'ang_d_error_mrad_s', ...
                  'duty_l_1000', 'duty_r_1000', 'enc_l', 'enc_r', ...
                  'len_current_mm', 'len_target_mm', 'delta_time', 'thinking_flag'};
        
        % CSVテーブル作成と保存
        T = array2table(data_buffer, 'VariableNames', header);
        writetable(T, csv_filename);
        
        fprintf('データを保存しました:\n');
        fprintf('  MAT: %s\n', mat_filename);
        fprintf('  CSV: %s\n', csv_filename);
        
    catch ME
        fprintf('データ保存エラー: %s\n', ME.message);
    end
end
