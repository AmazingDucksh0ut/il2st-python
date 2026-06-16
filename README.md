# il2st (Python) — 三菱 FX 指令表 → OpenPLC ST 轉譯器

將三菱 FX5U 指令表（Instruction List, IL）轉譯為 OpenPLC Runtime v3 可編譯執行的 IEC 61131-3 結構化文字（ST）。

本程式為碩士論文《基於開源架構之 PLC 模擬器建構方法》（張永傑，國立彰化師範大學電機與機械科技學系，2026）之成果，對應論文第 3.5 節（轉譯器設計）與 4.2 節。

## 需求

- Python 3.9 以上
- **零相依**：僅使用標準函式庫（argparse、csv、re、dataclasses 等），無須安裝任何第三方套件。

## 使用方式

命令列：

```
python il2st.py <input.csv> -o <output.st> -n <ProgramName> --task-ms 20
```

| 參數 | 說明 |
| --- | --- |
| `input.csv` | 輸入的指令表 CSV |
| `-o` | 輸出 .st 檔（省略則印至 stdout） |
| `-n` | ST 程式名稱（預設 `Prac`） |
| `--task-ms` | 掃描週期毫秒（預設 20） |

當作模組：

```python
from il2st import convert
st_text = convert(csv_text, program_name='Demo', task_ms=20)
```

## 輸入格式

三欄 CSV：`step, instr, operand`（標題列可有可無）。第三欄可用空白塞多個運算元，例如計時器 `OUT T0 K10` 寫成一欄 `T0 K10`。亦能自動辨識 GX Works3 的原生 IL 匯出檔（TSV）。

範例輸入：

```
step,instr,operand
0,LD,X0
1,OUT,Y0
```

核心轉譯結果為 `y0 := x0;`（外層會自動補上 `PROGRAM` / `VAR` 宣告與 `CONFIGURATION` 骨架）。

## 支援的指令

- 基本邏輯：LD、LDI、AND、ANI、OR、ORI、OUT、END
- 區塊／堆疊：ANB、ORB、MPS、MRD、MPP
- 邊緣偵測：PLS、PLF、LDP、LDF、ANDP、ANDF、ORP、ORF
- 自保持：SET、RST
- 計時／計數：OUT Tn（→ TON）、OUT Cn（→ CTU）
- 步進階梯：STL、RET（自動判別線性／並聯，見論文 3.5.4）
- 資料處理：MOV、INC、INCP、DEC、DECP、ZRST、CMP
- 比較接點：LD=、LD<、LD<= …（及 AND／OR 版）
- 特殊繼電器：SM8000/M8000、SM8002/M8002、SM8013/M8013

元件位址映射與型別對應見論文 3.5.5；輸出前會執行一道正確性檢查（lint），警告以註解附於檔末。

> 本轉譯器另有 **C# 等價版本**（論文第 5.4 節，對相同輸入逐位元組相同），以獨立 repo 釋出。

## 授權與來源

- 本程式以 **MIT License** 釋出（見 `LICENSE`）。
- 用於驗證本轉譯器的三十題實習，其題目出自教科書（石文傑、林家名、江宗霖，2021，《可程式控制器 PLC（含機電整合實務）》第四版，全華圖書，ISBN 978-986-5036-661-4），著作權屬原作者，**不在本開源範圍**。
