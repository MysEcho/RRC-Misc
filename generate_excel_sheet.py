import openpyxl
from openpyxl.styles import PatternFill, Font


def read_questions_from_file(file_path):
    questions = []
    start_index = 0

    with open(file_path, "r") as file:
        lines = file.readlines()
        while start_index < len(lines):
            end_index = start_index
            while end_index < len(lines) and lines[end_index].strip() != "":
                end_index += 1

            if end_index < len(lines):
                question_type = lines[start_index].strip()
                question = lines[start_index + 1].strip()
                options = []

                if question_type == "MC":
                    correct_option = lines[start_index + 2].strip().split(")")[0].strip()
                    for j in range(start_index + 3, start_index + 7):
                        option = lines[j].strip().split(")")[1].strip()
                        options.append(option)
                elif question_type == "TF":
                    correct_option = lines[start_index + 2].strip().split(")")[0].strip()
                    options.append(lines[start_index + 3].strip().split(")")[1].strip())
                    options.append(lines[start_index + 4].strip().split(")")[1].strip())
                    options.append("")
                    options.append("")
                elif question_type == "MR":
                    if start_index + 4 >= end_index:
                        raise Exception(
                            f"Not enough correct options for MR question starting at line {start_index + 1}"
                        )
                    correct_option = (
                        lines[start_index + 2].strip().split(")")[0].strip()
                        + ","
                        + lines[start_index + 3].strip().split(")")[0].strip()
                    )
                    for j in range(start_index + 4, start_index + 8):
                        option = lines[j].strip().split(")")[1].strip()
                        options.append(option)
                else:
                    raise Exception(f"Unknown question type '{question_type}' at line {start_index + 1}")

                questions.append((question_type, question, correct_option, options))

            start_index = end_index + 1

    return questions


def create_excel_quiz(questions):
    workbook = openpyxl.Workbook()
    sheet = workbook.active

    headers = ["QNo", "Question Type", "Question", "Correct Option", "Option 1", "Option 2", "Option 3", "Option 4"]
    for col, header in enumerate(headers, start=1):
        cell = sheet.cell(row=1, column=col)
        cell.value = header
        cell.font = Font(bold=True)
        cell.fill = PatternFill(start_color="C5D9F1", end_color="C5D9F1", fill_type="solid")

    for i, (question_type, question, correct_option, options) in enumerate(questions, start=1):
        sheet.cell(row=i + 1, column=1, value=i)
        sheet.cell(row=i + 1, column=2, value=question_type)
        sheet.cell(row=i + 1, column=3, value=question)

        if question_type == "TF":
            correct_option_num = 1 if correct_option == "a" else 2
        elif question_type == "MR":
            correct_options = correct_option.split(",")
            correct_option_num = "&".join(str(ord(opt.strip()[0]) - ord("a") + 1) for opt in correct_options)
        else:
            correct_option_num = ord(correct_option[0]) - ord("a") + 1
        sheet.cell(row=i + 1, column=4, value=correct_option_num)

        for j, option in enumerate(options, start=1):
            sheet.cell(row=i + 1, column=j + 4, value=option)

    for column in sheet.columns:
        max_length = 0
        column_letter = openpyxl.utils.get_column_letter(column[0].column)
        for cell in column:
            try:
                if len(str(cell.value)) > max_length:
                    max_length = len(str(cell.value))
            except:
                pass
        adjusted_width = max_length + 2
        sheet.column_dimensions[column_letter].width = adjusted_width

    workbook.save("Excel_Sheet.xlsx")
    print("Excel file has been created successfully!")


if __name__ == "__main__":
    questions = read_questions_from_file("questions.txt")
    create_excel_quiz(questions)
