import sys
import os
import time
import json
import requests
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QComboBox, QPushButton, QFileDialog, QSpinBox, QPlainTextEdit
)
import ollama

class OllamaUI(QWidget):
    def __init__(self):
        super().__init__()
        self.promptFilePath = None
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Ollama Prompt Runner")

        layout = QVBoxLayout()

        # Model selection label
        self.modelLabel = QLabel("Select Model:")
        layout.addWidget(self.modelLabel)

        # Model dropdown
        self.modelCombo = QComboBox()
        self.modelList = self.get_models()
        self.modelCombo.addItems(self.modelList)
        layout.addWidget(self.modelCombo)

        # Prompt file selection
        self.promptLabel = QLabel("Select Prompt File:")
        layout.addWidget(self.promptLabel)

        self.promptButton = QPushButton("Browse...")
        self.promptButton.clicked.connect(self.selectPromptFile)
        layout.addWidget(self.promptButton)

        # Number of runs
        self.runCountLabel = QLabel("Number of times to run:")
        layout.addWidget(self.runCountLabel)

        self.runCountSpin = QSpinBox()
        self.runCountSpin.setMinimum(1)
        self.runCountSpin.setMaximum(100)
        layout.addWidget(self.runCountSpin)

        # Run button
        self.runButton = QPushButton("Run")
        self.runButton.clicked.connect(self.runPrompts)
        layout.addWidget(self.runButton)

        # Log/Output display
        self.logBox = QPlainTextEdit()
        self.logBox.setReadOnly(True)
        layout.addWidget(self.logBox)

        self.setLayout(layout)

    def get_models(self):
        model_list = []
        raw_output = list(ollama.list())
        key, models = raw_output[0]
        for m in models:
            model_list.append(m.model) 
        return model_list

    def selectPromptFile(self):
        file_dialog = QFileDialog()
        file_dialog.setNameFilter("Text Files (*.txt)")
        if file_dialog.exec_():
            self.promptFilePath = file_dialog.selectedFiles()[0]
            self.promptLabel.setText(f"Selected Prompt File: {self.promptFilePath}")

    def runPrompts(self):
        selectedModel = self.modelCombo.currentText()
        if not self.promptFilePath:
            self.logBox.appendPlainText("Error: No prompt file selected.")
            return

        runCount = self.runCountSpin.value()

        # Read prompt
        try:
            with open(self.promptFilePath, 'r', encoding='utf-8') as f:
                promptText = f.read()
        except Exception as e:
            self.logBox.appendPlainText(f"Error reading prompt file: {e}")
            return

        # Create new folder (model + prompt name)
        promptName = os.path.splitext(os.path.basename(self.promptFilePath))[0]
        folderName = f"{selectedModel}_{promptName}"
        if not os.path.exists(folderName):
            os.makedirs(folderName)

        # We'll track basic stats for tokens/time
        totalTime = 0.0
        totalPromptTokens = 0
        totalOutputTokens = 0

        api_url = "http://localhost:11434/api/generate"
        headers = {
            "User-Agent": "Mozilla/5.0",
            "Referer": "http://example.com",
            "Content-Type": "application/json"
        }
        # Base data for requests
        base_data = {
            "model": selectedModel,
            "prompt": promptText,
            "stream": False
        }

        for i in range(runCount):
            self.logBox.appendPlainText(f"\n--- PROMPT TEST #{i+1} ---")
            startTime = time.time()
            try:
                response = requests.post(api_url, headers=headers, data=json.dumps(base_data))
                if response.status_code == 200:
                    # Parse the JSON response
                    resp_data = response.json()
                    text_response = resp_data.get("response", "")

                    # Time calculation
                    endTime = time.time()
                    duration = endTime - startTime
                    totalTime += duration

                    # Approx token counting
                    promptTokens = len(promptText.split())
                    outputTokens = len(text_response.split())
                    totalPromptTokens += promptTokens
                    totalOutputTokens += outputTokens

                    # Save output to file
                    outputFile = os.path.join(folderName, f"output_{i+1}.txt")
                    with open(outputFile, 'w', encoding='utf-8') as outF:
                        outF.write(text_response)

                    # Log success
                    self.logBox.appendPlainText(f"Success. Output saved to {outputFile}")
                else:
                    # Error in status code
                    self.logBox.appendPlainText(f"Error!!! Status code {response.status_code}")
                    self.logBox.appendPlainText(f"Details: {response.text}")
            except requests.exceptions.RequestException as e:
                self.logBox.appendPlainText(f"Exception in request: {str(e)}")

        # Final stats
        totalTokens = totalPromptTokens + totalOutputTokens
        if totalTime > 0:
            tps = totalTokens / totalTime
        else:
            tps = 0

        self.logBox.appendPlainText(
            f"\n--- Summary ---\n"
            f"Tokens/sec: {tps:.2f}\n"
            f"Prompt token length (approx): {totalPromptTokens}\n"
            f"Output token length (approx): {totalOutputTokens}"
        )

def main():
    app = QApplication(sys.argv)
    ui = OllamaUI()
    ui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
