#include <comn_pkg/drl_console.h>
#include <pthread.h>
#include <histedit.h>
#include <string>
using namespace std;
namespace DRLConsoleName
{
  string name = "test";
}

char * prompt(EditLine *e) {
  return (char* )(DRLConsoleName::name.c_str());
}

DRLConsole::DRLConsole(const char *consoleName, onCmdType fun)
{
  DRLConsoleName::name = string(consoleName) + string("> ");
  run_console_thread = true;
  console_thread = new boost::thread(boost::bind(&DRLConsole::console_thread_fun, this, fun));
}

DRLConsole::~DRLConsole()
{
  stopConsole();
  if (console_thread)
  {
    delete console_thread;
    console_thread = NULL;
  }

  history_end(myhist);
  el_end(el);
}

void DRLConsole::stopConsole()
{
    if (run_console_thread)
    {
        run_console_thread = false;
        pthread_cancel(console_thread->native_handle());
    }
}

void DRLConsole::console_thread_fun(onCmdType fun)
{
  const char *line;
  char buf[128];
  HistEvent ev;
  
 // std::cout << "begin console_thread_fun\n";
  
  el = el_init(DRLConsoleName::name.c_str(), stdin, stdout, stderr);
  el_set(el, EL_PROMPT, &prompt);//boost::bind(&WEConsole::prompt, this, _1));
  el_set(el, EL_EDITOR, "emacs");

  myhist = history_init();
  if(myhist == 0)
  {
//    std::cout << "history_init error\n";
  }
  history(myhist, &ev, H_SETSIZE, 800);
  el_set(el, EL_HIST, history, myhist);

  int count;
  while (run_console_thread)
  {
    line = el_gets(el, &count);
    if (count > 1)
    {
      strncpy(buf, line, count);
      buf[count - 1] = '\0';
      history(myhist, &ev, H_ENTER, line);
      fun((const char*) buf);
    }
  }
}

/*
char* CPPConsole::prompt(EditLine *e)
{
  return (char* )(CPPConsoleName::name.c_str());
}
*/
