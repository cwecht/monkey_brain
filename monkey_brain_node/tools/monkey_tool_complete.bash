#!/bin/bash
    
_monkey_tool()
{
    local cur=${COMP_WORDS[COMP_CWORD]}
    COMPREPLY=( $(compgen -W "list_io_plugins list_operator_plugins  show_io_plugin show_operator_plugin validate help" -- $cur) )
}
complete -F _monkey_tool monkey_tool
