import os

def combine_and_fix_txt_files(
    path_prefix,
    output_file_name,
    string_name
):
    """
    合并多个 _i.txt 文件并修复格式（确保每行有 expected_field_count 个字段）。
    
    参数:
    - path_prefix: 文件名前缀（如 "/path/to/file_prefix"，会自动拼接 _i.txt）
    - output_file_name: 合并输出文件名
    - index_range: 用于拼接文件名的 i 值范围，如 range(2, 9)
    - skip_index: 要跳过的索引列表（如 [5]）
    - expected_field_count: 每行应该有多少个字段，默认是8

    功能:
    - 合并文件内容
    - 修复每行字段数不符的问题（只保留合法行）
    """
    valid_lines = []
    for i in range(1, 9):
        file_path = f"{path_prefix}_{i}"+string_name+".txt"
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    fields = line.split()
                    if len(fields) == 8:
                        valid_lines.append(' '.join(fields))
                    else:
                        print(f"格式错误行（跳过）: '{line}'，字段数={len(fields)}")
            print(f"成功读取：{file_path}")
        except FileNotFoundError:
            print(f"文件不存在：{file_path}，跳过")
        except Exception as e:
            print(f"处理文件 {file_path} 时出错: {e}")

    # 写入输出文件
    with open("combined_txt/" + output_file_name, 'w') as out_f:
        for line in valid_lines:
            out_f.write(line + '\n')

    print(f"合并并修复完成，输出文件：{output_file_name}")


if __name__ == "__main__":

    for str in ['m', 'n', 'm_64']:
        # combine_and_fix_txt_files(path_prefix="../gt/gt_gps", output_file_name="gt.txt", string_name="")
        
        combine_and_fix_txt_files(path_prefix="../detect/detect", output_file_name="detect_" + str + ".txt", string_name=str)
        combine_and_fix_txt_files(path_prefix="../detect_convex/detect", output_file_name="detect_convex_" + str + ".txt", string_name=str)
        
        # combine_and_fix_txt_files(path_prefix="../DL/DL", output_file_name="DL_" + str + ".txt", string_name=str)
        combine_and_fix_txt_files(path_prefix="../regist/regist", output_file_name="regist_" + str + ".txt", string_name=str)
    
    
