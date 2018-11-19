# Kilka podstawowych komend gita

### Inicjalizowanie repozytorium:
- git init
	
### Klonowanie repozytorium:
- git clone https://USERNAME@bitbucket.org/TEAMNAME/REPO-NAME.git
- **git clone https://gregg127@bitbucket.org/roboticsfanatics/bazyliszek_arduino.git**
		
### Dodanie wszystkich plików z repo do commita: 
- git add *
	
### Commit:
- git commit -m "Wiadomosc do commita"
	
### Ustawienie nazwy uzytkownika do wysylanie repo na serwer
- git config user.name <nazwa uzytkownika>
- ** git config user.name gregg127 **
		
### Ustawienie emaila do wysylania repo na serwer
- git config user.email <email>
- **git config user.email s15045@pjwstk.edu.pl**
		
### Wyslanie zmian na serwer
- git push
	
### Pobranie zmian z serwera
- git pull

### Porzucenie swoich zmian
- git checkout *
- przydatne, gdy chcesz zrobic **git pull** bez dodawania swoich zmian (**merge**)
---

# Co wpisaæ?

**Wa¿ne: nazwa uzytkownika i email zgodne z kontem bitbucket**

1. git config user.name <nazwa uzytkownika>
2. git config user.email <email>
3. git clone https://<nazwa uzytkownika>@bitbucket.org/roboticsfanatics/bazyliszek_arduino.git
4. praca:
	- dodanie / edycja / jakiejs funkcjonalnosc
	- **git add ***
	- **git commit -m "opis zmian"**
5. wrzucenie zmian na serwer **przed tym upewnij sie, ze zmiany zosta³y zacommitowane**
	- **git push**
6. pobranie zmian z serwera
	- **git pull**
