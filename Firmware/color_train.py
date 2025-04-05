import pandas as pd
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler

# 1. Excelファイルの読み込み
df = pd.read_excel("color.xlsx")  # 適宜パスを修正してください

# 2. 特徴量とラベルを分離
X = df[['r', 'g', 'b', 'w']]
y = df['label']

# 3. スケーリング（標準化）
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# 4. マルチクラスロジスティック回帰モデルの学習
model = LogisticRegression(multi_class='multinomial', solver='lbfgs')
model.fit(X_scaled, y)

# 5. スケール項とバイアス項を係数に含める
#    (x - mean)/scale に対して w を学習しているため、元のスケールに戻す
original_weights = model.coef_ / scaler.scale_
adjusted_intercepts = model.intercept_ - np.dot(model.coef_, scaler.mean_ / scaler.scale_)

# 6. 回帰係数 + バイアスを1つの行列にまとめる
adjusted_coefficients = np.hstack([original_weights, adjusted_intercepts.reshape(-1, 1)])

# 7. 結果をDataFrameで表示
columns = ['r', 'g', 'b', 'w', 'bias']
result_df = pd.DataFrame(adjusted_coefficients, columns=columns)
print(result_df)
