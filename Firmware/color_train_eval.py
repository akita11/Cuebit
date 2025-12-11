import sys
import pandas as pd
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler

def main():
    # ---------------------------------------------------------
    # 1. 引数のチェック (入力CSVファイル指定)
    # ---------------------------------------------------------
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <input_csv_file>")
        sys.exit(1)

    file_path = sys.argv[1]

    # ---------------------------------------------------------
    # 2. CSVファイルの読み込み
    # ---------------------------------------------------------
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        sys.exit(1)

    # 必須カラムの確認
    required_cols = ['r', 'g', 'b', 'w', 'label']
    if not all(col in df.columns for col in required_cols):
        print(f"Error: CSV must contain columns: {required_cols}")
        sys.exit(1)

    # ---------------------------------------------------------
    # 3. 学習プロセス
    # ---------------------------------------------------------
    X = df[['r', 'g', 'b', 'w']]
    y = df['label']

    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    model = LogisticRegression(multi_class='multinomial', solver='lbfgs')
    model.fit(X_scaled, y)

    # ---------------------------------------------------------
    # 4. 生データ用の係数変換
    # ---------------------------------------------------------
    # weights: (coef / scale)
    # bias: intercept - sum(coef * mean / scale)
    raw_weights = model.coef_ / scaler.scale_
    raw_biases = model.intercept_ - np.dot(model.coef_, scaler.mean_ / scaler.scale_)

    # 配列結合: [w_r, w_g, w_b, w_w, bias]
    # shape: (n_classes, 5)
    adjusted_coefficients = np.hstack([raw_weights, raw_biases.reshape(-1, 1)])
    
    class_names = model.classes_
    n_classes = len(class_names)

    # ---------------------------------------------------------
    # 5. C言語形式での係数出力
    # ---------------------------------------------------------
    print("\n/* --- C Language Output Start --- */")
    print("// Class mapping:")
    for i, cls_name in enumerate(class_names):
        print(f"// Index {i}: {cls_name}")
    
    print("")
    # C配列の定義を出力
    print(f"float weights[{n_classes}][5] = {{")
    
    for i in range(n_classes):
        # 各要素を文字列化 (精度6桁などを指定)
        vals = [f"{v:.6f}" for v in adjusted_coefficients[i]]
        line = ", ".join(vals)
        
        # 最後の行以外はカンマをつける
        comma = "," if i < n_classes - 1 else ""
        print(f"  {{{line}}}{comma}")
        
    print("};")
    print("/* --- C Language Output End --- */\n")

    # ---------------------------------------------------------
    # 6. 評価 (Cコードのロジックを模倣して検証)
    # ---------------------------------------------------------
    print("--- Verifying Accuracy with C-logic ---")
    
    X_values = X.values  # [[r, g, b, w], ...]
    y_values = y.values
    correct_count = 0
    total_samples = len(X_values)

    for i in range(total_samples):
        # float input[4] = {R, G, B, W};
        input_vec = X_values[i] 
        
        # float scores[n_classes];
        scores = [0.0] * n_classes
        
        # C言語のループ構造を再現
        # for (int c = 0; c < 5; c++)
        for c in range(n_classes):
            # scores[c] = weights[c][4]; // bias
            # バイアスは配列の最後の要素 (インデックス4)
            bias_val = adjusted_coefficients[c][4]
            scores[c] = bias_val
            
            # for (int k = 0; k < 4; k++)
            #   scores[c] += input[k] * weights[c][k];
            for k in range(4):
                weight_val = adjusted_coefficients[c][k]
                scores[c] += input_vec[k] * weight_val

        # int maxIndex = 0;
        # for (int c = 1; c < 5; c++) ...
        max_index = 0
        for c in range(1, n_classes):
            if scores[c] > scores[max_index]:
                max_index = c
        
        # マッピングを使ってラベル名に変換
        predicted_label = class_names[max_index]
        
        if predicted_label == y_values[i]:
            correct_count += 1

    accuracy = correct_count / total_samples
    print(f"Accuracy: {accuracy:.4f} ({correct_count}/{total_samples})")

if __name__ == "__main__":
    main()
