import customtkinter as ctk
import pandas as pd
import tkinter as tk
from tkinter import messagebox

def dateCheck(date):
    # Checks if the inserted date is an valid layout and returns a counter == 10 if its correct
    date = str(date)
    counter = 0
    numbers = [0, 1, 3, 4, 6, 7, 8, 9]     #indexes that should have a digit
    slashes = [2, 5]                       #indexes that should have a /
    date_model = ""

    for date_model in date:
        if (date_model.isdigit() and counter in numbers):
            counter += 1
        elif (date_model == "/" and counter in slashes):
            counter += 1
        else:
            counter = 11
            break

    return counter

class App(ctk.CTk):

    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        self.geometry("1080x750")
        self.title("Software de controle de dados dos pacientes")
        self.iconbitmap("C:\\Software_controle_pacientes\\Imagens\\icone.ico")

        self.file = pd.read_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = self.file.set_index('Nome')
        self.found = self.file

        self.list_content = tk.StringVar(value="")

        self.main_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.main_frame.pack(expand=True, fill="both")

        self.name_label = ctk.CTkLabel(self.main_frame, text="Nome do paciente", font=("Roboto", 12))
        self.name_label.pack(padx=10)

        self.name_entry = ctk.CTkEntry(self.main_frame)
        self.name_entry.pack(padx=10, pady=10)

        self.cpf_label = ctk.CTkLabel(self.main_frame, text="CPF do paciente", font=("Roboto", 12))
        self.cpf_label.pack(padx=10)

        self.cpf_entry = ctk.CTkEntry(self.main_frame)
        self.cpf_entry.pack(padx=10, pady=10)

        self.payday_label = ctk.CTkLabel(self.main_frame, text="Dia do pagamento do paciente", font=("Roboto", 12))
        self.payday_label.pack(padx=10)

        self.payday_entry = ctk.CTkEntry(self.main_frame)
        self.payday_entry.pack(padx=10, pady=10)

        self.birth_label = ctk.CTkLabel(self.main_frame, text="Data de nascimento do paciente", font=("Roboto", 12))
        self.birth_label.pack(padx=10)

        self.birth_entry = ctk.CTkEntry(self.main_frame)
        self.birth_entry.pack(padx=10, pady=10)

        self.listBox = tk.Listbox(self.main_frame, width=100, height=14, listvariable=self.list_content, font=("Arial", 16))
        self.listBox.pack(padx=10, pady=5)
        self.listBox.insert(tk.END, f"Nome      CPF      Dia de pagamento      Data de nascimento")

        self.warning_label = ctk.CTkLabel(self.main_frame, text="", font=("Arial", 24), text_color="red")
        self.warning_label.pack(padx=20, pady=20)

        button_frame1 = ctk.CTkFrame(self.main_frame, width=720, height=500)
        button_frame1.pack(padx=20, pady=10)

        self.add_button = ctk.CTkButton(button_frame1, text="Adicionar infos", command=self.addInfo)
        self.add_button.pack(side="left",padx=25)

        self.search_button = ctk.CTkButton(button_frame1, text="Procurar", command=self.searchInfo)
        self.search_button.pack(side="left", padx=25)

        self.backup_button = ctk.CTkButton(button_frame1, text="Backup", command=self.backupCommand)
        self.backup_button.pack(side="left", padx=25)

        self.delete_button = ctk.CTkButton(button_frame1, text="Deletar infos", command=self.deleteInfo)
        self.delete_button.pack(side="right", padx=25)

        self.edit_button = ctk.CTkButton(button_frame1, text="Editar infos", command=self.editInfo)
        self.edit_button.pack(padx=25, side="left")

        button_frame2 = ctk.CTkFrame(self.main_frame, width=720, height=500)
        button_frame2.pack(padx=20, pady=10)

        self.confirm_button = ctk.CTkButton(button_frame2, text="Confirmar alterações", state="disabled", command=self.confirmChanges)
        self.confirm_button.pack(side="left", padx=25)

        self.cancel_button = ctk.CTkButton(button_frame2, text="Cancelar alterações", state="disabled", command=self.cancelChanges)
        self.cancel_button.pack(side="right", padx=25)

    def listbox_to_listOfDict(self):
        # Turns listbox line into a list of dictionaries
        self.file = self.file.reset_index()

        full_string = str(self.listBox.get(self.listBox.curselection()))
        full_string = full_string.replace("     ", "")

        #creates a string of the name from the listbox text
        res = ""
        num = "0123456789"
        for i in full_string:
            if i in num:
                break
            else:
                res+=i

        selected_item = self.file.loc[self.file['Nome'].str.contains(res, case=False)]
        selected_item = selected_item.to_dict('records')

        return selected_item
    
    def infoCheck(self):
        # Checks if the users input is ok
        name = self.name_entry.get()
        cpf = self.cpf_entry.get()
        payday = self.payday_entry.get()
        birth = self.birth_entry.get()

        # Checks if inserted cpf has the normal amount of characters that a cpf should have
        cpf_size = 0
        for i in cpf:
            cpf_size += 1
            
        if (birth == "" or payday == "" or cpf == "" or name == ""):
            self.warning_label.configure(text="Insira dados para todas as entradas")
            return 0

        if not (cpf.isdigit()):
            self.warning_label.configure(text="O CPF deve conter apenas números")
            return 0
        
        if not ((self.file.loc[self.file['CPF'].astype(str).str.contains(cpf, case=False)]).empty):
            self.warning_label.configure(text="O CPF inserido já consta nos dados dos pacientes, ou não contém a quantidade correta de caracteres")
            return 0
        
        if (cpf_size != 11):
            self.warning_label.configure(text="O CPF inserido não contém a quantidade correta de caracteres")
            return 0
        
        if (dateCheck(payday) != 10):
            self.warning_label.configure(text="O dia de pagamento inserida não possui um formato válido, o correto deve ser no estilo 00/00/0000")
            return 0            
        
        if (dateCheck(birth) != 10):
            self.warning_label.configure(text="A data de nascimento inserida não possui um formato válido, o correto deve ser no estilo 00/00/0000")
            return 0            

        full_data = [{
                    "Nome": name,
                    "CPF": cpf,
                    "Dia de pagamento": payday,
                    "Data de nascimento": birth}]
        
        return full_data
    
    def addInfo(self):
        # Inserts the users input into the csv file and into the listbox
        self.warning_label.configure(text="")

        full_data = self.infoCheck()
        if (full_data == 0):
            return 0
        full_df = pd.DataFrame(full_data)
        full_df = full_df.set_index('Nome')
        self.file = pd.concat([self.file, full_df])

        self.listBox.delete(1, "end")
        self.listBox.insert(tk.END, f"{full_data[0]['Nome']}     {full_data[0]['CPF']}     {full_data[0]['Dia de pagamento']}     {full_data[0]['Data de nascimento']}")

        self.file = self.file.to_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = pd.read_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = self.file.set_index('Nome')

        self.name_entry.delete(0, "end")
        self.cpf_entry.delete(0, "end")
        self.payday_entry.delete(0, "end")
        self.birth_entry.delete(0, "end")

        self.found = full_df    
        self.found = self.found.to_dict('records')

    def searchInfo(self):
        # Gets the users entry and searches for similar infos in the data frame
        self.warning_label.configure(text="")

        self.file = self.file.reset_index()

        name = self.name_entry.get()
        cpf = str(self.cpf_entry.get())
        payday = str(self.payday_entry.get())
        birth = str(self.birth_entry.get())

        if (name != ""):
            self.found = self.file.loc[self.file['Nome'].str.contains(name, case=False)]

        elif (cpf != ""):
            self.found = self.file.loc[self.file['CPF'].astype(str).str.contains(cpf, case=False)]  

        elif (payday != ""):
            self.found = self.file.loc[self.file['Dia de pagamento'].astype(str).str.contains(payday, case=False)]     

        elif (birth != ""):
            self.found = self.file.loc[self.file['Data de nascimento'].astype(str).str.contains(birth, case=False)]       

        else:
            self.warning_label.configure(text="Insira dados para pelo menos uma entrada")
            self.file = self.file.set_index('Nome')
            return 0
        
        self.found = self.found.to_dict('records')

        self.listBox.delete(1, "end")

        for i in range(len(self.found)):
            self.listBox.insert(tk.END, f"{self.found[i]['Nome']}     {self.found[i]['CPF']}    {self.found[i]['Dia de pagamento']}       {self.found[i]['Data de nascimento']}")

        self.file = self.file.set_index('Nome')

    def editInfo(self):
        # Get selected item from the listbox and puts it in the entry boxes
        self.warning_label.configure(text="")

        if (self.listBox.curselection() == () or self.listBox.curselection() == (0,)):
            self.warning_label.configure(text="Selecione um paciente para editar suas informações")
            return 0
        
        self.found = self.listbox_to_listOfDict()

        nome = str(self.found[0]['Nome'])
        cpf = str(self.found[0]["CPF"])
        payday = str(self.found[0]['Dia de pagamento'])
        birth = str(self.found[0]['Data de nascimento'])

        self.name_entry.delete(0, "end")
        self.cpf_entry.delete(0, "end")
        self.payday_entry.delete(0, "end")
        self.birth_entry.delete(0, "end")

        self.name_entry.insert(0, nome)
        self.cpf_entry.insert(0, cpf)
        self.payday_entry.insert(0, payday)
        self.birth_entry.insert(0, birth)

        self.add_button.configure(state="disabled")
        self.edit_button.configure(state="disabled")
        self.search_button.configure(state="disabled")
        self.delete_button.configure(state="disabled")
        self.backup_button.configure(state="disabled")

        self.confirm_button.configure(state="normal")
        self.cancel_button.configure(state="normal")

        self.file = self.file.set_index('Nome')

    def cancelChanges(self):
        # Removes the contente from the entry boxes and returns to previous state
        self.warning_label.configure(text="")

        self.name_entry.delete(0, "end")
        self.cpf_entry.delete(0, "end")
        self.payday_entry.delete(0, "end")
        self.birth_entry.delete(0, "end")

        self.add_button.configure(state="normal")
        self.edit_button.configure(state="normal")
        self.search_button.configure(state="normal")
        self.delete_button.configure(state="normal")
        self.backup_button.configure(state="normal")

        self.confirm_button.configure(state="disabled")
        self.cancel_button.configure(state="disabled")

    def confirmChanges(self):
        # Exclude previous infos of the data frame and add the new modified info
        aux_df = self.file
        self.file = self.file[self.file['CPF'] != self.found[0]['CPF']]

        full_data = self.infoCheck()
        if (full_data == 0):
            self.file = aux_df         # In case there is a misstype in the users input it will return the previous info to the df
            return 0

        self.addInfo()

        self.file = self.file.to_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = pd.read_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = self.file.set_index('Nome')

        self.add_button.configure(state="normal")
        self.edit_button.configure(state="normal")
        self.search_button.configure(state="normal")
        self.delete_button.configure(state="normal")
        self.backup_button.configure(state="normal")

        self.confirm_button.configure(state="disabled")
        self.cancel_button.configure(state="disabled")

    def deleteInfo(self):
        # Gets the list box chosen info and deletes it from the data frame
        self.warning_label.configure(text="")
        
        if (self.listBox.curselection() == () or self.listBox.curselection() == (0,)):
            self.warning_label.configure(text="Selecione um paciente para deletar suas informações")
            return 0
        
        self.found = self.listbox_to_listOfDict()

        if messagebox.askokcancel(title="Excluir paciente?", message=f"Tem certeza que quer excluir {self.found[0]['Nome']}?"):
            pass
        else:
            self.warning_label.configure(text="Operação cancelada")
            self.file = self.file.set_index('Nome')
            return 0

        self.file = self.file.set_index('Nome')
        self.file = self.file[self.file['CPF'] != self.found[0]['CPF']]

        self.file = self.file.to_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = pd.read_csv("C:\\Software_controle_pacientes\\Tabelas\\Tabela_Pacientes.csv")
        self.file = self.file.set_index('Nome')

        self.listBox.delete(1, "end")
        self.listBox.insert(tk.END, f"{self.found[0]['Nome']} foi removido(a)")

        self.name_entry.delete(0, "end")
        self.cpf_entry.delete(0, "end")
        self.payday_entry.delete(0, "end")
        self.birth_entry.delete(0, "end")

    def backupCommand(self):
        # Copy the file to a new file
        self.warning_label.configure(text="")

        if messagebox.askokcancel(title="Fazer backup?", message="Essa ação irá subsituir o backup antigo pelo arquivo atual, tem certeza que quer continuar?"):
            self.file = self.file.to_csv("C:\\Software_controle_pacientes\\Tabelas\\Copia_Tabela_Pacientes.csv")
            self.warning_label.configure(text="Operação realizada")
        else:
            self.warning_label.configure(text="Operação cancelada")

app = App()
app.mainloop()
