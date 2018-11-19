### Inicjalizowanie repozytorium:
	git init
	
### Klonowanie repozytorium:
	git clone https://USERNAME@bitbucket.org/TEAMNAME/REPO-NAME.git
	**git clone https://gregg127@bitbucket.org/roboticsfanatics/bazyliszek_arduino.git**
		
### Dodanie wszystkich plików z repo do commita: 
	git add *
	
### Commit:
	git commit -m "Wiadomosc do commita"
	
### Ustawienie nazwy uzytkownika do wysylanie repo na serwer
	git config user.name <nazwa uzytkownika>
	** git config user.name gregg127 **
		
### Ustawienie emaila do wysylania repo na serwer
	git config user.email <email>
	**git config user.email s15045@pjwstk.edu.pl**
		
### Wyslanie zmian na serwer
	git push