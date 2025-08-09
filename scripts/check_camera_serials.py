#!/usr/bin/env python3
# =============================================================================
# Fluent Vision - RealSenseカメラシリアル番号確認スクリプト
# =============================================================================
# 
# 概要：
#   - 接続されているRealSenseカメラのシリアル番号を確認
#   - 現在の設定ファイルとの整合性をチェック
#   - カメラの種類（D415/D405）を自動判別
#
# 作者：Takashi Otsuka
# 作成日：2024年
# バージョン：1.0
#
# 使用方法：
#   python3 check_camera_serials.py
# =============================================================================

import pyrealsense2 as rs
import yaml
import os

def main():
    """
    メイン関数：RealSenseカメラのシリアル番号を確認し、設定ファイルとの整合性をチェック
    """
    print("🔍 RealSenseカメラのシリアル番号を確認中...")
    print("")
    
    # RealSenseコンテキストを初期化
    ctx = rs.context()
    devices = ctx.query_devices()
    
    print(f"接続されているRealSenseデバイス数: {len(devices)}台")
    print("")
    
    # 各デバイスの情報を表示
    for i, device in enumerate(devices):
        # デバイス情報を取得
        name = device.get_info(rs.camera_info.name)           # カメラ名
        serial = device.get_info(rs.camera_info.serial_number) # シリアル番号
        firmware = device.get_info(rs.camera_info.firmware_version) # ファームウェアバージョン
        
        print(f"デバイス {i}:")
        print(f"  名前: {name}")
        print(f"  シリアル番号: {serial}")
        print(f"  ファームウェア: {firmware}")
        
        # カメラタイプを判別して設定ファイルの場所を表示
        if "D415" in name:
            print(f"  ➡️  D415カメラです。シリアル番号を fv_realsense_d415.yaml に設定してください")
        elif "D405" in name:
            print(f"  ➡️  D405カメラです。シリアル番号を fv_realsense_d405.yaml に設定してください")
        else:
            print(f"  ⚠️  未対応のカメラタイプです: {name}")
        print("")
    
    print("=" * 60)
    print("")
    print("現在の設定ファイル状況:")
    print("")
    
    # 設定ファイルの現在の値を確認
    check_config_file("D415", "fv_realsense_d415.yaml")
    check_config_file("D405", "fv_realsense_d405.yaml")
    
    print("=" * 60)
    print("")
    print("💡 設定ファイルの更新が必要な場合:")
    print("   ./update_camera_serials.sh を実行してください")
    print("")

def check_config_file(camera_type, config_filename):
    """
    設定ファイルのシリアル番号を確認する関数
    
    Args:
        camera_type (str): カメラタイプ（"D415" または "D405"）
        config_filename (str): 設定ファイル名
    """
    # スクリプトのディレクトリを基準に相対パスで設定
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, config_filename)
    
    try:
        # 設定ファイルを読み込み
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            
        # シリアル番号を取得
        serial_number = config['/**']['ros__parameters']['camera_selection']['serial_number']
        print(f"{camera_type}設定ファイル ({config_filename}):")
        print(f"  設定されているシリアル番号: {serial_number}")
        
        # ファイルの存在確認
        if os.path.exists(config_path):
            print(f"  ✅ 設定ファイルは存在します")
        else:
            print(f"  ❌ 設定ファイルが見つかりません")
            
    except FileNotFoundError:
        print(f"{camera_type}設定ファイル ({config_filename}):")
        print(f"  ❌ 設定ファイルが見つかりません: {config_path}")
    except KeyError as e:
        print(f"{camera_type}設定ファイル ({config_filename}):")
        print(f"  ⚠️ 設定ファイルの構造が不正です: {e}")
    except Exception as e:
        print(f"{camera_type}設定ファイル ({config_filename}):")
        print(f"  ❌ 設定ファイルの読み込みエラー: {e}")
    
    print("")

if __name__ == "__main__":
    main()