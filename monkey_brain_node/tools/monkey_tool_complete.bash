#!/bin/bash
    
_monkey_tool()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W "list_io_plugins list_operator_plugins list_decision_engine_plugins show_io_plugin show_operator_plugin tist_available_values validate help" -- $cur) )
}
complete -F _monkey_tool monkey_tool
