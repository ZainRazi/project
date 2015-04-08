# encoding: utf-8
import('../type/selector')
import('../type/variator')
import('../type/breeder')

register t(:simple) {
  extends(:breeder)

  indexed_collection :selectors, :selector, i(:selector)
  indexed_collection :variators, :variator, i(:variator)

  composer do
    s = spec(:breeder, :simple).extend

    # Build the constructor.
    cons = [
      "self.selectors = {}",
      "self.variators = {}"
    ]

    # Inject the name of each operator into its instance.
    (selectors.pairs + variators.pairs).each do |n, op|
      op.set(:name, n.to_s)
    end

    # Inject the selectors into the constructor and add them as
    # dependencies to this specification.
    selectors.each_pair do |n, sel|
      si_sel = sel.compose
      s.dependency(si_sel.specification)
      cons << "self.selectors['#{n}'] = #{si_sel}"
    end

    # Construct the variators in the correct order.
    #
    # NOTE: This approach is probably quite poor for efficiency, but since
    # we're dealing with such a small number of operators, its simplicity
    # more than justifies its usage.
    sorted_variators = lambda { |vars| 
      i = 0
      while i < vars.length
        gt_i = vars.index { |j| vars[i][1].get(:name) == vars[i][1].get(:source) }
        if !gt_i.nil? && gt_i > i
          vars[gt_i], vars[i] = vars[i], vars[gt_i]
        else
          i += 1
        end
      end
      return vars
    }.call(variators.pairs)

    # Construct the set of terminal variators.
    terminal_variators = []
    sorted_variators.each_with_index do |v, i|
      k1, v1 = v
      break unless sorted_variators[i ... -1].detect { |k2, v2|
        v1.get(:name) == v2.get(:source)
      }.nil?
      terminal_variators << k1
    end

    # Construct each of the variators and add them as dependencies to
    # this specification.
    cons << ""
    sorted_variators.each do |n, v|
      src = v.get(:source)
      if variators.key?(src)
        v.set(:source, Code["self.variators['#{src}']"])
      else
        v.set(:source, Code["self.selectors['#{src}']"])
      end
      si_v = v.compose
      s.dependency(si_v.specification)
      cons << "self.variators['#{n}'] = #{si_v}"
    end

    # Inject the list of terminal variators into the constructor.
    cons << ""
    terminal_variators = terminal_variators.map { |v|
      "self.variators['#{v}']"
    }.join(', ')
    cons << "self.terminals = [#{terminal_variators}]"

    # Put together the constructor.
    s.constructor([], source: cons.join("\n"))

    # Construct the prepare buffer method.
    s.method(:prepare_buffer, returns: s.map(s.ruby(:string), s.list(s.s(:individual))),
      accepts: [
        s.parameter(:rng,         s.s(:rng)),
        s.parameter(:statistics,  s.s(:statistics)),
        s.parameter(:deme,        s.s(:deme))
    ], source: (selectors.pairs.map { |n, s|
        "  '#{n.to_s}' => selectors['#{n.to_s}'].prepare(rng, statistics, deme.contents)"
      } + variators.pairs.map { |n, v|
        "  '#{n.to_s}' => []"
    }).join(",\n").prepend("return {\n").concat("\n}"))

    # Return the constructed specification.
    s

  end

}

register s(:simple) {
  extends(:breeder)

  attribute :selectors, map(ruby(:string), s(:selector))
  attribute :variators, map(ruby(:string), s(:variator))
  attribute :terminals, list(s(:variator))

  implement_method :breed do
    buffer = prepare_buffer(rng, statistics, deme)
    return Array.new(deme.num_offspring) do
     terminals.sample.produce(rng, statistics, buffer, 1)[0]
    end
  end

}
